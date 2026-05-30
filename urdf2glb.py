#!/usr/bin/env python3
"""
URDF + Collada DAE → GLB converter with proper node hierarchy.
No trimesh dependency for export — builds GLTF JSON + binary buffer directly.
"""
import sys, os, struct, json, base64
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import OrderedDict
import numpy as np

# ── DAE parser (no pycollada needed, just XML) ──
def parse_dae(dae_path):
    """Extract vertex positions and triangle indices from Collada file."""
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    tree = ET.parse(dae_path)
    root = tree.getroot()

    meshes = []
    for geom in root.findall('.//c:library_geometries/c:geometry', ns):
        mesh_el = geom.find('c:mesh', ns)
        if mesh_el is None:
            continue

        # Find position source
        vertices_el = mesh_el.find('c:vertices', ns)
        if vertices_el is None:
            continue
        input_el = vertices_el.find('c:input[@semantic="POSITION"]', ns)
        if input_el is None:
            continue
        source_id = input_el.get('source', '').lstrip('#')

        # Find the float array for positions
        pos_source = mesh_el.find(f'c:source[@id="{source_id}"]', ns)
        if pos_source is None:
            continue
        float_array = pos_source.find('c:float_array', ns)
        accessor = pos_source.find('c:technique_common/c:accessor', ns)

        if float_array is None or accessor is None:
            continue

        count = int(accessor.get('count', 0))
        stride = int(accessor.get('stride', 3))
        verts = np.array([float(x) for x in float_array.text.split()], dtype=np.float32)
        verts = verts.reshape(count, stride)[:, :3]  # take xyz

        # Get triangle indices
        triangles_el = mesh_el.find('c:triangles', ns)
        if triangles_el is not None:
            p_input = triangles_el.find('c:input[@semantic="VERTEX"]', ns)
            if p_input is not None:
                p_offset = int(p_input.get('offset', 0))
                p_el = triangles_el.find('c:p', ns)
                if p_el is not None:
                    all_indices = np.array([int(x) for x in p_el.text.split()], dtype=np.uint32)
                    num_inputs = len(triangles_el.findall('c:input', ns))
                    faces = all_indices[p_offset::num_inputs].reshape(-1, 3)
                    meshes.append({'vertices': verts, 'faces': faces})

    if not meshes:
        return None

    # Merge all meshes from this file
    all_verts = []
    all_faces = []
    voff = 0
    for m in meshes:
        all_verts.append(m['vertices'])
        all_faces.append(m['faces'] + voff)
        voff += len(m['vertices'])
    return {'vertices': np.vstack(all_verts), 'faces': np.vstack(all_faces)}


# ── URDF parser ──
def parse_urdf(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    urdf_dir = Path(urdf_path).parent

    links = {}
    joints = []

    for link in root.findall('link'):
        name = link.get('name')
        visual = link.find('visual')
        mesh_ref = None
        origin_xyz = [0,0,0]
        origin_rpy = [0,0,0]

        if visual is not None:
            origin_el = visual.find('origin')
            if origin_el is not None:
                origin_xyz = [float(v) for v in origin_el.get('xyz', '0 0 0').split()]
                origin_rpy = [float(v) for v in origin_el.get('rpy', '0 0 0').split()]
            geom = visual.find('geometry')
            if geom is not None:
                mesh_el = geom.find('mesh')
                if mesh_el is not None:
                    mesh_path = str((urdf_dir / mesh_el.get('filename')).resolve())
                    scale = [float(s) for s in mesh_el.get('scale', '1 1 1').split()]
                    mesh_ref = {'path': mesh_path, 'scale': scale}

        links[name] = {'mesh': mesh_ref, 'xyz': origin_xyz, 'rpy': origin_rpy}

    for joint in root.findall('joint'):
        jtype = joint.get('type', 'fixed')
        name = joint.get('name')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        origin_el = joint.find('origin')
        xyz = [0,0,0]
        rpy = [0,0,0]
        if origin_el is not None:
            xyz = [float(v) for v in origin_el.get('xyz', '0 0 0').split()]
            rpy = [float(v) for v in origin_el.get('rpy', '0 0 0').split()]
        joints.append({'name': name, 'parent': parent, 'child': child,
                       'xyz': xyz, 'rpy': rpy, 'type': jtype})

    return links, joints


def rpy_to_mat4(rpy, xyz):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ], dtype=np.float32)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


# ── GLB Builder ──
class GLBBuilder:
    def __init__(self):
        self.nodes = []           # list of node dicts
        self.meshes = []          # list of mesh dicts
        self.accessors = []       # list of accessor dicts
        self.bufferViews = []     # list of bufferView dicts
        self.buffers = []         # list of bytearrays
        self.mesh_index = {}      # link_name -> mesh index
        self.node_index = {}      # link_name -> node index
        self.root_node = None

    def add_mesh(self, vertices, faces, node_name, parent_name=None):
        """Add a mesh primitive and create a node for it."""
        if node_name in self.node_index:
            # Node already exists — just set parent
            if parent_name:
                self.set_parent(node_name, parent_name)
            return self.node_index[node_name]
        verts = vertices.astype(np.float32)
        indices = faces.astype(np.uint32)
        verts_bytes = verts.tobytes()
        indices_bytes = indices.tobytes()

        # Pad to 4-byte alignment
        while len(verts_bytes) % 4: verts_bytes += b'\x00'
        while len(indices_bytes) % 4: indices_bytes += b'\x00'

        buf = self.buffers[0] if self.buffers else bytearray()
        if not self.buffers:
            self.buffers.append(buf)

        # Vertex buffer view
        vbv_offset = len(buf)
        buf.extend(verts_bytes)
        vbv_len = len(verts_bytes)
        self.bufferViews.append({
            'buffer': 0, 'byteOffset': vbv_offset, 'byteLength': vbv_len,
            'target': 34962  # ARRAY_BUFFER
        })

        # Index buffer view
        ibv_offset = len(buf)
        buf.extend(indices_bytes)
        ibv_len = len(indices_bytes)
        self.bufferViews.append({
            'buffer': 0, 'byteOffset': ibv_offset, 'byteLength': ibv_len,
            'target': 34963  # ELEMENT_ARRAY_BUFFER
        })

        # Accessors
        min_vals = verts.min(axis=0).tolist()
        max_vals = verts.max(axis=0).tolist()
        self.accessors.append({
            'bufferView': len(self.bufferViews) - 2,
            'componentType': 5126,  # FLOAT
            'count': len(verts),
            'type': 'VEC3',
            'min': min_vals,
            'max': max_vals,
        })
        pos_acc = len(self.accessors) - 1

        self.accessors.append({
            'bufferView': len(self.bufferViews) - 1,
            'componentType': 5125,  # UNSIGNED_INT
            'count': len(indices) * 3,
            'type': 'SCALAR',
        })
        idx_acc = len(self.accessors) - 1

        mi = len(self.meshes)
        self.meshes.append({
            'primitives': [{
                'attributes': {'POSITION': pos_acc},
                'indices': idx_acc,
                'mode': 4,  # TRIANGLES
            }]
        })

        # Node
        ni = len(self.nodes)
        self.nodes.append({'name': node_name, 'mesh': mi})
        self.node_index[node_name] = ni
        self.mesh_index[node_name] = mi

        return ni

    def add_empty_node(self, name):
        if name in self.node_index:
            return self.node_index[name]
        ni = len(self.nodes)
        self.nodes.append({'name': name})
        self.node_index[name] = ni
        return ni

    def add_node(self, name):
        """Alias: ensure node exists (no mesh)."""
        return self.add_empty_node(name)

    def set_parent(self, child_name, parent_name):
        ci = self.node_index[child_name]
        pi = self.node_index.get(parent_name)
        if pi is not None:
            self.nodes[ci]['_parent'] = pi

    def build_children(self):
        """Convert _parent to children arrays, build root list."""
        children = [[] for _ in range(len(self.nodes))]
        for i, n in enumerate(self.nodes):
            p = n.pop('_parent', None)
            if p is not None:
                children[p].append(i)

        root_nodes = []
        for i in range(len(self.nodes)):
            if children[i]:
                self.nodes[i]['children'] = children[i]
            # Root: node not referenced as child by anyone, and no _parent
        # Find roots
        is_child = set()
        for n in self.nodes:
            for c in n.get('children', []):
                is_child.add(c)
        for i in range(len(self.nodes)):
            if i not in is_child:
                root_nodes.append(i)

        return root_nodes

    def export(self, path):
        roots = self.build_children()
        root_node_idx = roots[0] if roots else 0

        gltf = {
            'asset': {'version': '2.0'},
            'scene': 0,
            'scenes': [{'nodes': [root_node_idx]}],
            'nodes': self.nodes,
            'meshes': self.meshes,
            'accessors': self.accessors,
            'bufferViews': self.bufferViews,
            'buffers': [{'byteLength': len(self.buffers[0])}],
        }

        json_str = json.dumps(gltf, separators=(',', ':'), ensure_ascii=False)
        # Pad JSON to 4-byte alignment
        while len(json_str) % 4:
            json_str += ' '
        json_bytes = json_str.encode('utf-8')

        bin_data = bytes(self.buffers[0])

        # GLB header
        total_len = 12 + 8 + len(json_bytes) + 8 + len(bin_data)
        glb = struct.pack('<I', 0x46546C67)  # magic
        glb += struct.pack('<I', 2)           # version
        glb += struct.pack('<I', total_len)   # total length

        # JSON chunk
        glb += struct.pack('<I', len(json_bytes))
        glb += struct.pack('<I', 0x4E4F534A)  # 'JSON'
        glb += json_bytes

        # BIN chunk
        glb += struct.pack('<I', len(bin_data))
        glb += struct.pack('<I', 0x004E4942)  # 'BIN\0'
        glb += bin_data

        with open(path, 'wb') as f:
            f.write(glb)
        print(f"  Exported {path} ({len(glb)/1e6:.1f} MB)")


# ── Main ──
def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <urdf_path> [output.glb]")
        sys.exit(1)

    urdf_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else Path(urdf_path).with_suffix('.glb')

    print(f"Parsing URDF: {urdf_path}")
    links, joints = parse_urdf(urdf_path)
    print(f"  {len(links)} links, {len(joints)} joints")

    # Load meshes
    mesh_cache = {}
    for name, link in links.items():
        if link['mesh'] is None:
            continue
        path = link['mesh']['path']
        if path in mesh_cache:
            continue
        print(f"  Loading {Path(path).name}...")
        mesh_data = parse_dae(path)
        if mesh_data is not None:
            mesh_cache[path] = mesh_data
            print(f"    {len(mesh_data['vertices'])} verts, {len(mesh_data['faces'])} faces")
        else:
            print(f"    WARN: failed to parse")

    # Build GLB — single pass, recursive tree walk
    builder = GLBBuilder()

    # Find root (link not a child of any joint)
    all_children = {j['child'] for j in joints}
    root_link = None
    for name in links:
        if name not in all_children:
            root_link = name
            break

    # Build children map for joints
    children_of = {}
    for j in joints:
        children_of.setdefault(j['parent'], []).append(j)

    def build_link(link_name, parent_node_name=None):
        """Recursively add a link and its children to the GLB."""
        link = links[link_name]

        if link['mesh'] is not None and link['mesh']['path'] in mesh_cache:
            mesh_data = mesh_cache[link['mesh']['path']]
            scale = link['mesh']['scale']
            verts = mesh_data['vertices'].copy()
            verts[:,0] *= scale[0]; verts[:,1] *= scale[1]; verts[:,2] *= scale[2]
            T_link = rpy_to_mat4(link.get('rpy',[0,0,0]), link.get('xyz',[0,0,0]))
            ones = np.ones((len(verts), 1), dtype=np.float32)
            verts = (T_link @ np.hstack([verts, ones]).T).T[:, :3]
            node_idx = builder.add_mesh(verts, mesh_data['faces'], link_name, parent_node_name)
        else:
            node_idx = builder.add_empty_node(link_name)

        # Process child joints: apply joint transform to child link's mesh
        for cj in children_of.get(link_name, []):
            child_name = cj['child']
            child_link = links[child_name]
            Tj = rpy_to_mat4(cj['rpy'], cj['xyz'])

            if child_link['mesh'] is not None and child_link['mesh']['path'] in mesh_cache:
                mesh_data = mesh_cache[child_link['mesh']['path']]
                scale = child_link['mesh']['scale']
                verts = mesh_data['vertices'].copy()
                verts[:,0] *= scale[0]; verts[:,1] *= scale[1]; verts[:,2] *= scale[2]
                Tcl = rpy_to_mat4(child_link.get('rpy',[0,0,0]), child_link.get('xyz',[0,0,0]))
                T = Tj @ Tcl
                ones = np.ones((len(verts), 1), dtype=np.float32)
                verts = (T @ np.hstack([verts, ones]).T).T[:, :3]
                child_idx = builder.add_mesh(verts, mesh_data['faces'], child_name, link_name)
            else:
                child_idx = builder.add_empty_node(child_name)

            # Recurse into grandchildren
            build_link(child_name, link_name)

    build_link(root_link)
    builder.export(output_path)


if __name__ == '__main__':
    main()

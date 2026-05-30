#!/usr/bin/env python3
"""
URDF + Collada DAE → GLB converter with proper kinematic node hierarchy.

Architecture:
  Layer 1 (this script): DAE raw mesh → link-local coords → GLB with joint tree
  Layer 2 (GLB file):     Standard glTF 2.0, nodes = kinematic chain, vertices in link-local
  Layer 3 (viewer):       Load GLB, animate joints, apply SLAM pose at runtime

Transform chain per link:
  raw_dae_verts → [DAE scene node transform] → [URDF visual origin] → link-local coords
  GLTF node matrix = joint origin (kinematic chain only)

Dependencies: trimesh, pycollada, numpy
"""

import sys, os
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import OrderedDict

import numpy as np
import trimesh
from collada import Collada


# ═══════════════════════════════════════════════════════════════
# Utility: DAE scene node transform extraction
# ═══════════════════════════════════════════════════════════════

def get_dae_scene_transform(dae_path: str) -> np.ndarray:
    """
    Extract the 4x4 transform matrix from the DAE's first geometry-bearing
    scene node. Returns identity if no transform found.

    DAE files from DCC tools often have scene-node transforms that orient
    the mesh from the modeler's coordinate system to the intended visual
    frame. We must apply this before further processing.
    """
    try:
        doc = Collada(dae_path, ignore=[Collada.options.noImageLoading])
    except Exception:
        return np.eye(4, dtype=np.float32)

    scene = doc.scene
    if scene is None:
        return np.eye(4, dtype=np.float32)

    for node in scene.nodes:
        # Find first node that has geometry children
        has_geometry = any(
            hasattr(c, 'geometry') and c.geometry is not None
            for c in node.children
        )
        if has_geometry:
            for transform in node.transforms:
                # pycollada wraps transforms; MatrixTransform has .matrix
                if hasattr(transform, 'matrix'):
                    return np.array(transform.matrix, dtype=np.float32)
            break

    return np.eye(4, dtype=np.float32)


# ═══════════════════════════════════════════════════════════════
# URDF parsing
# ═══════════════════════════════════════════════════════════════

def rpy_to_mat4(rpy, xyz):
    """Convert roll-pitch-yaw (radians) + translation to 4x4 matrix."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ], dtype=np.float32)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3] = np.array(xyz, dtype=np.float32)
    return T


def parse_urdf(urdf_path: str):
    """
    Parse URDF and return (links, joints, root_link_name).

    links: {name: {mesh_path, visual_origin_xyz, visual_origin_rpy, scale}}
    joints: [{name, parent, child, xyz, rpy, type, axis}]
    """
    tree = ET.parse(urdf_path)
    root_el = tree.getroot()
    urdf_dir = Path(urdf_path).parent

    links = OrderedDict()
    joints = []

    for link in root_el.findall('link'):
        name = link.get('name')
        visual = link.find('visual')
        mesh_path = None
        vis_xyz = np.zeros(3, dtype=np.float32)
        vis_rpy = np.zeros(3, dtype=np.float32)
        scale = np.ones(3, dtype=np.float32)

        if visual is not None:
            origin_el = visual.find('origin')
            if origin_el is not None:
                vis_xyz = np.array(
                    [float(v) for v in origin_el.get('xyz', '0 0 0').split()],
                    dtype=np.float32
                )
                vis_rpy = np.array(
                    [float(v) for v in origin_el.get('rpy', '0 0 0').split()],
                    dtype=np.float32
                )
            geom = visual.find('geometry')
            if geom is not None:
                mesh_el = geom.find('mesh')
                if mesh_el is not None:
                    mesh_path = str((urdf_dir / mesh_el.get('filename')).resolve())
                    scale = np.array(
                        [float(s) for s in mesh_el.get('scale', '1 1 1').split()],
                        dtype=np.float32
                    )

        links[name] = {
            'mesh_path': mesh_path,
            'vis_xyz': vis_xyz,
            'vis_rpy': vis_rpy,
            'scale': scale,
        }

    for joint in root_el.findall('joint'):
        jtype = joint.get('type', 'fixed')
        name = joint.get('name')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        origin_el = joint.find('origin')
        xyz = np.zeros(3, dtype=np.float32)
        rpy = np.zeros(3, dtype=np.float32)
        if origin_el is not None:
            xyz = np.array(
                [float(v) for v in origin_el.get('xyz', '0 0 0').split()],
                dtype=np.float32
            )
            rpy = np.array(
                [float(v) for v in origin_el.get('rpy', '0 0 0').split()],
                dtype=np.float32
            )
        axis_el = joint.find('axis')
        axis = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        if axis_el is not None:
            axis = np.array(
                [float(v) for v in axis_el.get('xyz', '1 0 0').split()],
                dtype=np.float32
            )

        joints.append({
            'name': name,
            'parent': parent,
            'child': child,
            'xyz': xyz,
            'rpy': rpy,
            'type': jtype,
            'axis': axis,
        })

    # Find root link (not a child of any joint)
    all_children = {j['child'] for j in joints}
    root_link = None
    for name in links:
        if name not in all_children:
            root_link = name
            break

    return links, joints, root_link


# ═══════════════════════════════════════════════════════════════
# Mesh loading + transform baking (to link-local frame)
# ═══════════════════════════════════════════════════════════════

def load_and_transform_mesh(link_info: dict, extra_tf: np.ndarray = None) -> trimesh.Trimesh | None:
    """
    Load a DAE mesh and apply transforms to bring it into link-local coordinates.

    Pipeline:
      raw_dae → scale → DAE_scene_tx → URDF_visual_origin → extra_tf → link-local

    Returns a trimesh.Trimesh or None.
    """
    mesh_path = link_info['mesh_path']
    if mesh_path is None or not os.path.exists(mesh_path):
        return None

    # Load with trimesh
    try:
        loaded = trimesh.load(mesh_path, force='mesh')
    except Exception as e:
        print(f"    WARN: trimesh failed to load {Path(mesh_path).name}: {e}")
        return None

    # Extract the first Trimesh (DAE may contain multiple geometries/scene nodes)
    if isinstance(loaded, trimesh.Scene):
        if len(loaded.geometry) == 0:
            return None
        # Take the first mesh geometry
        geom = list(loaded.geometry.values())[0]
        if not isinstance(geom, trimesh.Trimesh):
            return None
        mesh = geom.copy()
    elif isinstance(loaded, trimesh.Trimesh):
        mesh = loaded.copy()
    else:
        return None

    if len(mesh.vertices) == 0:
        return None

    # 1) Apply scale
    scale = link_info['scale']
    if not np.allclose(scale, 1.0):
        mesh.vertices[:, 0] *= scale[0]
        mesh.vertices[:, 1] *= scale[1]
        mesh.vertices[:, 2] *= scale[2]

    # 2) Get DAE scene transform
    dae_scene_tf = get_dae_scene_transform(mesh_path)

    # 3) Get URDF visual origin transform
    vis_tf = rpy_to_mat4(link_info['vis_rpy'], link_info['vis_xyz'])

    # 4) Combined transform: DAE_scene_tf first, then visual_origin on top
    combined_tf = vis_tf @ dae_scene_tf

    # 5) Apply extra transform (e.g. rear leg 180° Y flip)
    if extra_tf is not None and not np.allclose(extra_tf, np.eye(4)):
        combined_tf = extra_tf @ combined_tf

    # Apply to vertices
    if not np.allclose(combined_tf, np.eye(4)):
        verts = mesh.vertices
        ones = np.ones((len(verts), 1), dtype=np.float32)
        transformed = (combined_tf @ np.hstack([verts, ones]).T).T[:, :3]
        mesh.vertices = transformed.astype(np.float32)

    return mesh


# ═══════════════════════════════════════════════════════════════
# Scene graph construction
# ═══════════════════════════════════════════════════════════════

def build_scene_graph(links, joints, root_link, mesh_cache):
    """
    Build a trimesh.Scene with the full kinematic tree.

    Strategy:
      - Mesh-bearing links: scene.add_geometry() creates a node with geometry
      - Meshless links: scene.graph.update() creates an empty transform frame
      - Children reference their parent by link_name (frame or geometry node)

    Joint origins become 4x4 node transform matrices.
    Parent-child relationships follow the URDF joint tree.

    Returns (scene, joint_info_map)
    """
    scene = trimesh.Scene()

    # Build parent→child map
    children_of = {}
    for j in joints:
        children_of.setdefault(j['parent'], []).append(j)

    # Joint info for each link (what joint connects it to its parent)
    joint_for_link = {}
    for j in joints:
        joint_for_link[j['child']] = j

    # ── Build tree bottom-up: leaves first ──
    # We need to know which links need frames (no mesh but have children)
    needs_frame = set()
    for link_name, link in links.items():
        if link['mesh_path'] is None:
            # Check if it has children
            if link_name in children_of:
                needs_frame.add(link_name)

    # Also root needs a frame if it has no mesh
    if links[root_link]['mesh_path'] is None:
        needs_frame.add(root_link)

    # Track created nodes
    created = set()
    geom_count = 0

    def build_node(link_name, parent_name):
        """Create node for this link. Return the node name used."""
        nonlocal geom_count

        if link_name in created:
            return link_name

        link = links[link_name]

        # Compute joint transform
        node_tf = np.eye(4, dtype=np.float32)
        if link_name in joint_for_link:
            j = joint_for_link[link_name]
            node_tf = rpy_to_mat4(j['rpy'], j['xyz'])

        frame_from = parent_name if parent_name else 'world'

        if link['mesh_path'] is not None and link_name in mesh_cache and mesh_cache[link_name] is not None:
            # Has mesh: use add_geometry
            scene.add_geometry(
                mesh_cache[link_name],
                node_name=link_name,
                parent_node_name=parent_name if parent_name else None,
                geom_name=link_name + '_geom',
                transform=node_tf,
            )
            geom_count += 1
        elif link_name in needs_frame:
            # No mesh but has children: create empty frame
            scene.graph.update(
                frame_to=link_name,
                frame_from=frame_from,
                matrix=node_tf,
            )
        # else: no mesh and no children → skip entirely

        created.add(link_name)

        # Recurse into children
        for cj in children_of.get(link_name, []):
            build_node(cj['child'], link_name)

        return link_name

    build_node(root_link, None)

    print(f"  Scene graph: {geom_count} geometry nodes, "
          f"{len(scene.graph.transforms.edge_data)} transform edges, "
          f"{len(needs_frame)} empty frames")

    return scene, joint_for_link


# ═══════════════════════════════════════════════════════════════
# Mesh deduplication: same DAE file → shared trimesh
# ═══════════════════════════════════════════════════════════════

def build_mesh_cache(links):
    """
    Load and transform meshes, deduplicating by (dae_path, vis_xyz, vis_rpy, scale) key.
    Returns {link_name: transformed_trimesh_or_None}
    """
    key_to_mesh = {}
    mesh_for_link = {}

    for name, link in links.items():
        if link['mesh_path'] is None:
            mesh_for_link[name] = None
            continue


        mesh_path = link['mesh_path']

        key = (
            mesh_path,
            tuple(round(v, 6) for v in link['vis_xyz']),
            tuple(round(v, 6) for v in link['vis_rpy']),
            tuple(round(v, 6) for v in link['scale']),
        )

        if key not in key_to_mesh:
            print(f"  Loading {Path(mesh_path).name} for {name}...", end=' ')
            mesh = load_and_transform_mesh(link)
            key_to_mesh[key] = mesh
            if mesh is not None:
                print(f"{len(mesh.vertices)} verts, {len(mesh.faces)} faces")
            else:
                print("SKIP (no mesh data)")
        else:
            mesh = key_to_mesh[key]
            print(f"  Reusing {Path(link['mesh_path']).name} for {name}")

        mesh_for_link[name] = key_to_mesh[key]

    return mesh_for_link


# ═══════════════════════════════════════════════════════════════
# GLB Export (using trimesh built-in)
# ═══════════════════════════════════════════════════════════════

def export_glb(scene, output_path, joint_for_link):
    """Export trimesh Scene to GLB, then annotate with joint metadata."""
    print(f"\nExporting GLB to {output_path}...")

    # trimesh exports GLTF with node transforms preserved
    scene.export(output_path, file_type='glb')

    size_mb = os.path.getsize(output_path) / 1e6
    print(f"  Done: {size_mb:.1f} MB")

    # Verify and report
    import json, struct
    with open(output_path, 'rb') as f:
        data = f.read()
    json_len = struct.unpack('<I', data[12:16])[0]
    gltf = json.loads(data[20:20 + json_len])

    nodes_with_mesh = sum(1 for n in gltf['nodes'] if 'mesh' in n)
    nodes_with_transform = sum(
        1 for n in gltf['nodes']
        if any(k in n for k in ['matrix', 'translation', 'rotation', 'scale'])
    )
    print(f"  Nodes: {len(gltf['nodes'])} ({nodes_with_mesh} with mesh, "
          f"{nodes_with_transform} with transform)")
    print(f"  Meshes: {len(gltf['meshes'])}")

    return gltf


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <urdf_path> [output.glb]")
        print(f"Example: {sys.argv[0]} models/go1/go1.urdf models/go1/go1.glb")
        sys.exit(1)

    urdf_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else str(Path(urdf_path).with_suffix('.glb'))

    print(f"╔══════════════════════════════════════╗")
    print(f"║   urdf2glb — URDF + DAE → GLB       ║")
    print(f"╚══════════════════════════════════════╝")
    print(f"\nURDF: {urdf_path}")
    print(f"Output: {output_path}\n")

    # 1. Parse URDF
    print("── Parsing URDF ──")
    links, joints, root_link = parse_urdf(urdf_path)
    print(f"  Links: {len(links)}, Joints: {len(joints)}, Root: {root_link}")

    # Count mesh-bearing links
    mesh_links = sum(1 for l in links.values() if l['mesh_path'])
    print(f"  Links with meshes: {mesh_links}")

    # Report joints by type
    revolute = sum(1 for j in joints if j['type'] == 'revolute')
    fixed = sum(1 for j in joints if j['type'] == 'fixed')
    print(f"  Revolute joints: {revolute}, Fixed joints: {fixed}")

    # 2. Load and transform meshes (deduped)
    print("\n── Loading meshes ──")
    mesh_for_link = build_mesh_cache(links)

    # 3. Build scene graph
    print("\n── Building scene graph ──")
    scene, joint_for_link = build_scene_graph(links, joints, root_link, mesh_for_link)

    # 4. Export GLB
    print("\n── Exporting ──")
    gltf = export_glb(scene, output_path, joint_for_link)

    # 5. Summary
    print(f"\n✓ Done! Output: {output_path}")

    # Print the kinematic tree
    children_of = {}
    for j in joints:
        children_of.setdefault(j['parent'], []).append(j)

    def print_tree(name, depth=0):
        prefix = "  " * depth + ("└─ " if depth > 0 else "")
        j = joint_for_link.get(name, {})
        jtype = j.get('type', 'root')
        axis = j.get('axis', None)
        axis_str = f" axis=({axis[0]:.1f},{axis[1]:.1f},{axis[2]:.1f})" if axis is not None and jtype == 'revolute' else ""
        has_mesh = "●" if links[name]['mesh_path'] else "○"
        print(f"{prefix}{has_mesh} {name} [{jtype}]{axis_str}")
        for cj in children_of.get(name, []):
            print_tree(cj['child'], depth + 1)

    print("\nKinematic tree:")
    print_tree(root_link)


if __name__ == '__main__':
    main()

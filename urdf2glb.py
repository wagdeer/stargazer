#!/usr/bin/env python3
"""
Convert URDF + Collada (DAE) meshes to a single GLB file for three.js.
No urdf-loader dependency — one GLTFLoader.load() call.

Usage: python3 urdf2glb.py models/go1/go1.urdf models/go1/go1.glb
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import OrderedDict
import trimesh
import numpy as np
import json
import struct
import base64

URDF_NS = {'xacro': 'http://www.ros.org/wiki/xacro'}

def parse_urdf(urdf_path):
    """Extract joint tree and link mesh references from URDF."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    urdf_dir = Path(urdf_path).parent

    links = {}
    joints = []
    joint_map = {}  # child_link -> joint_name

    for link in root.findall('link'):
        name = link.get('name')
        visual = link.find('visual')
        mesh = None
        origin_xyz = [0,0,0]
        origin_rpy = [0,0,0]

        if visual is not None:
            origin_el = visual.find('origin')
            if origin_el is not None:
                xyz = origin_el.get('xyz', '0 0 0')
                rpy = origin_el.get('rpy', '0 0 0')
                origin_xyz = [float(v) for v in xyz.split()]
                origin_rpy = [float(v) for v in rpy.split()]
            geom = visual.find('geometry')
            if geom is not None:
                mesh_el = geom.find('mesh')
                if mesh_el is not None:
                    mesh_path = mesh_el.get('filename')
                    scale = mesh_el.get('scale', '1 1 1')
                    mesh_path = str((urdf_dir / mesh_path).resolve())
                    mesh = {'path': mesh_path, 'scale': [float(s) for s in scale.split()]}

        links[name] = {'mesh': mesh, 'xyz': origin_xyz, 'rpy': origin_rpy}

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
        joint_map[child] = name

    return links, joints, joint_map


def rpy_to_matrix(rpy):
    """Convert roll-pitch-yaw to 4x4 transform matrix."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])
    return R


def load_mesh_cache(urdf_dir, links):
    """Load each unique DAE mesh, convert to trimesh, return cached dict."""
    cache = {}
    for link_name, link in links.items():
        if link['mesh'] is None:
            continue
        path = link['mesh']['path']
        if path in cache:
            continue
        try:
            mesh = trimesh.load(path, force='mesh')
            if isinstance(mesh, trimesh.Scene):
                # flatten scene to single mesh
                meshes = []
                for g in mesh.geometry.values():
                    if hasattr(g, 'vertices'):
                        meshes.append(g)
                if meshes:
                    mesh = trimesh.util.concatenate(meshes)
                else:
                    mesh = trimesh.Trimesh()
            cache[path] = mesh
            print(f"  Loaded {Path(path).name}: {len(mesh.vertices)} verts, {len(mesh.faces)} faces")
        except Exception as e:
            print(f"  WARN: Failed to load {path}: {e}")
            cache[path] = trimesh.Trimesh()
    return cache


def build_gltf(links, joints, joint_map, mesh_cache, output_path):
    """Build GLB binary from hierarchy."""
    # Find root link (not a child of any joint)
    all_children = {j['child'] for j in joints}
    root_link = None
    for name in links:
        if name not in all_children:
            root_link = name
            break

    if root_link is None:
        root_link = list(links.keys())[0]

    # Build parent->children map
    children_of = {}
    for j in joints:
        parent = j['parent']
        children_of.setdefault(parent, []).append(j)

    # Use trimesh's scene export
    scene = trimesh.Scene()
    
    def add_link(link_name, parent_node=None):
        link = links[link_name]

        has_mesh = link['mesh'] is not None
        mesh_node = None

        if has_mesh:
            path = link['mesh']['path']
            scale = link['mesh']['scale']
            mesh = mesh_cache.get(path)
            if mesh is not None and len(mesh.vertices) > 0:
                m = mesh.copy()
                m.apply_scale(scale)
                R_link = rpy_to_matrix(link['rpy'])
                T_link_local = np.eye(4)
                T_link_local[:3,:3] = R_link
                T_link_local[:3,3] = link['xyz']
                m.apply_transform(T_link_local)
                mesh_node = scene.add_geometry(m, node_name=link_name, parent_node_name=parent_node)
            else:
                mesh_node = scene.add_geometry(trimesh.Trimesh(), node_name=link_name, parent_node_name=parent_node)
        else:
            mesh_node = scene.add_geometry(trimesh.Trimesh(), node_name=link_name, parent_node_name=parent_node)

        # Process child joints: apply joint transform, add child link directly
        child_joints = children_of.get(link_name, [])
        for cj in child_joints:
            Rj = rpy_to_matrix(cj['rpy'])
            Tj = np.eye(4)
            Tj[:3,:3] = Rj
            Tj[:3,3] = cj['xyz']

            child_link = cj['child']
            cl = links[child_link]
            # Merge joint transform with child link's origin
            Rcl = rpy_to_matrix(cl['rpy'])
            Tcl = np.eye(4)
            Tcl[:3,:3] = Rcl
            Tcl[:3,3] = cl['xyz']
            T = Tj @ Tcl

            if cl['mesh'] is not None:
                path = cl['mesh']['path']
                mesh = mesh_cache.get(path)
                if mesh is not None and len(mesh.vertices) > 0:
                    m = mesh.copy()
                    m.apply_scale(cl['mesh']['scale'])
                    m.apply_transform(T)
                    scene.add_geometry(m, node_name=child_link, parent_node_name=link_name)

            # Recurse into grandchildren
            add_link(child_link, parent_node=link_name)

    add_link(root_link)
    scene.export(output_path, file_type='glb')
    print(f"\nExported {output_path}")
    return True


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <urdf_path> [output.glb]")
        sys.exit(1)

    urdf_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else Path(urdf_path).with_suffix('.glb')

    urdf_dir = Path(urdf_path).parent

    print(f"Parsing URDF: {urdf_path}")
    links, joints, joint_map = parse_urdf(urdf_path)
    print(f"  {len(links)} links, {len(joints)} joints")

    print("Loading meshes...")
    mesh_cache = load_mesh_cache(urdf_dir, links)

    print("Building GLB...")
    build_gltf(links, joints, joint_map, mesh_cache, output_path)


if __name__ == '__main__':
    main()

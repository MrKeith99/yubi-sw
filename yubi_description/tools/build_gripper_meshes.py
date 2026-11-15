#!/usr/bin/env python3
"""Visual meshes of the YUBI gripper for the URDF, from the yubi-hw assembly STEP.

usage: build_gripper_meshes.py "<yubi-hw>/STEP/gripper/YUBI Gripper Assy_Dynamixel_ver2.STEP" <meshes dir>
needs: cadquery-ocp (< 7.9), numpy-stl is not needed (binary STL written here)

Frame: <side>_hand_root of yubi_hand.macro.xacro (x along the fingers, z up through the camera).
In the assembly the finger pivots are at X = 839.5 / 869.5, Z = 1270.5 mm, and the finger mid
plane is Y = 854.5 mm; the root is 16 mm ahead of the pivots (finger joint origin x = -0.016).
Parts are grouped by material so the URDF can colour them:
  gripper_body.STL    printed parts (PLA black)
  gripper_servo.STL   DYNAMIXEL XM430-W350-R
  gripper_metal.STL   bracket, shafts, gears, bearings
  camera_elp.STL      ELP-USBFHD01M-L180, in the <side>_hand_cam_link frame (root + 0.0426 m in z)
Left out: fingers, pads, flaps (the finger meshes cover them), the UR5e flange and all fasteners.
"""
import struct
import sys

from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool

ROOT_MM = (854.5, 854.5, 1286.5)      # root origin in the assembly (X, Y, Z)
CAM_Z = 0.0426                        # hand_cam_link above the root [m]
BODY = ('BRACKET_DYNAMIXEL', 'CASE', 'UPPER PLATE_CAMERA BKT')
SERVO = ('DYNAMIXEL',)
CAMERA = ('USB CAMERA',)
SKIP = ('FINGER', 'PAD_', 'RUBBER SHEET', 'FLAP', 'UR5e_FLANGE',
        # bolts, inserts, pins, keys, washers: small, and their threads cost most of the triangles
        'CB', 'SB-', 'JP', 'KEG', 'WSSAB', 'XDSHC')


def to_root(p):
    """Assembly mm -> root m: x = +Z, y = +X, z = +Y."""
    return ((p[2] - ROOT_MM[2]) / 1000.0, (p[0] - ROOT_MM[0]) / 1000.0, (p[1] - ROOT_MM[1]) / 1000.0)


def triangles(shape):
    BRepMesh_IncrementalMesh(shape, 0.1, False, 0.3, True)
    out = []
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face_s(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            pts = [tri.Node(i).Transformed(trsf) for i in range(1, tri.NbNodes() + 1)]
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if face.Orientation() == TopAbs_REVERSED:
                    b, c = c, b
                out.append([(pts[k - 1].X(), pts[k - 1].Y(), pts[k - 1].Z()) for k in (a, b, c)])
        ex.Next()
    return out


def write_stl(path, tris, shift_z=0.0):
    with open(path, 'wb') as f:
        f.write(b'YUBI gripper visual mesh (from yubi-hw STEP)'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for t in tris:
            v = [to_root(p) for p in t]
            v = [(x, y, z - shift_z) for x, y, z in v]
            ux, uy, uz = (v[1][i] - v[0][i] for i in range(3))
            wx, wy, wz = (v[2][i] - v[0][i] for i in range(3))
            n = (uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx)
            m = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5 or 1.0
            f.write(struct.pack('<12fH', *(c / m for c in n), *v[0], *v[1], *v[2], 0))
    print('{}: {} triangles'.format(path, len(tris)))


def main():
    step, out = sys.argv[1:3]
    doc = TDocStd_Document(TCollection_ExtendedString('yubi'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.ReadFile(step)
    reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(label):
        n = TDataStd_Name()
        return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), n) else ''

    free = TDF_LabelSequence()
    tool.GetFreeShapes(free)
    parts = TDF_LabelSequence()
    tool.GetComponents_s(free.Value(1), parts)
    groups = {'gripper_body': [], 'gripper_servo': [], 'gripper_metal': [], 'camera_elp': []}
    for i in range(1, parts.Length() + 1):
        label, ref = parts.Value(i), TDF_Label()
        part = name(ref) if tool.GetReferredShape_s(label, ref) else name(label)
        if any(part.startswith(s) or s in part for s in SKIP):
            continue
        tris = triangles(tool.GetShape_s(label))
        if not tris:
            continue
        key = ('gripper_body' if part.startswith(BODY) else 'gripper_servo' if part.startswith(SERVO)
               else 'camera_elp' if part.startswith(CAMERA) else 'gripper_metal')
        groups[key] += tris
        print('  {:<40} -> {} ({})'.format(part, key, len(tris)))
    for key, tris in groups.items():
        write_stl('{}/{}.STL'.format(out, key), tris, CAM_Z if key == 'camera_elp' else 0.0)


if __name__ == '__main__':
    main()

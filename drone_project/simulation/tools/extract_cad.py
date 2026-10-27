#!/usr/bin/env python3
"""Optional OCP-based reproduction of CAD geometry inventory; no mass inference."""
import argparse
import hashlib
import json
from pathlib import Path
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID


def extract(path):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone: raise ValueError('STEP read failed')
    reader.TransferRoots()
    shape = reader.OneShape()
    bounds = Bnd_Box(); BRepBndLib.AddOptimal_s(shape, bounds)
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    solids = []
    while explorer.More():
        props = GProp_GProps(); BRepGProp.VolumeProperties_s(explorer.Current(), props)
        box = Bnd_Box(); BRepBndLib.AddOptimal_s(explorer.Current(), box)
        solids.append({'bounds_mm':box.Get(), 'volume_mm3':props.Mass(),
                       'centroid_mm':props.CentreOfMass().Coord()})
        explorer.Next()
    return {'source':path.name, 'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'bounds_mm':bounds.Get(), 'solids':solids,
            'limitation':'Not an all-up flight assembly; no densities assigned to unlabelled solids'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('step', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mesh', type=Path, help='Optional binary STL visual, scaled from mm to m')
    args = parser.parse_args(); args.output.write_text(json.dumps(extract(args.step), indent=2)+'\n')
    if args.mesh:
        from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
        from OCP.BRepMesh import BRepMesh_IncrementalMesh
        from OCP.StlAPI import StlAPI_Writer
        from OCP.gp import gp_Trsf, gp_Pnt
        reader=STEPControl_Reader(); reader.ReadFile(str(args.step)); reader.TransferRoots()
        transform=gp_Trsf(); transform.SetScale(gp_Pnt(0,0,0),0.001)
        shape=BRepBuilderAPI_Transform(reader.OneShape(),transform,True).Shape()
        BRepMesh_IncrementalMesh(shape,0.0002,False,0.2,True)
        args.mesh.parent.mkdir(parents=True,exist_ok=True)
        writer=StlAPI_Writer(); writer.ASCIIMode=False
        if not writer.Write(shape,str(args.mesh)): raise RuntimeError('STL export failed')

#!/usr/bin/env python3
"""分析 FreeCAD CAD 文件的手臂零件结构"""
import sys
sys.path.insert(0, '/tmp/.mount_FreeCAxxxx/usr/lib')  # ignore

# FreeCAD needs to be imported specially
import FreeCAD, Part, Mesh, os

cad_path = '/mnt/data2/projects/xiaozhi/xiaozhi-electronbot-docs/docs/cad/cadelectron.FCStd'

doc = FreeCAD.open(cad_path)
shell = doc.getObject('_X2_52A05DE54EF6_X0_')
print(f'Root: {shell.Name}, children: {len(shell.OutList)}')

left_parts = ['Part__Feature042','Part__Feature045','Part__Feature046','Part__Feature047','Part__Feature048','Part__Feature049']
right_parts = ['Part__Feature027','Part__Feature028','Part__Feature029','Part__Feature030','Part__Feature031','Part__Feature032','Part__Feature033']

for label, parts in [('LEFT_ARM', left_parts), ('RIGHT_ARM', right_parts)]:
    print(f'\n=== {label} ===')
    for name in parts:
        obj = doc.getObject(name)
        if not obj:
            print(f'  {name}: NOT FOUND')
            continue
        try:
            bb = obj.Shape.BoundBox
            vol = obj.Shape.Volume
            print(f'  {name}: vol={vol:.0f}mm³  BB(X={bb.XMin:.0f}~{bb.XMax:.0f}, Y={bb.YMin:.0f}~{bb.YMax:.0f}, Z={bb.ZMin:.0f}~{bb.ZMax:.0f})')
        except Exception as e:
            print(f'  {name}: error={e}')

FreeCAD.closeDocument(doc.Name)
print('\nDone.')

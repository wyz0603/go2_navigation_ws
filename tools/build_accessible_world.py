#!/usr/bin/env python3
"""Build a separate three-level navigation scene with matching visual/collision geometry."""
from pathlib import Path
import xml.etree.ElementTree as ET
import math
root=Path(__file__).resolve().parents[1]
tree=ET.parse(root/'src/robot_sim_worlds/worlds/building_obstacles.sdf')
world=tree.getroot().find('world');world.set('name','building_accessible')
for child in list(world):
 if child.tag in ('model','include'):world.remove(child)
def box(name,x,y,z,sx,sy,sz,pitch=0,roll=0):
 model=ET.SubElement(world,'model',name=name);ET.SubElement(model,'static').text='true'
 ET.SubElement(model,'pose').text=f'{x} {y} {z} {roll} {pitch} 0'
 link=ET.SubElement(model,'link',name='body')
 for tag in ('collision','visual'):
  item=ET.SubElement(link,tag,name=tag);geo=ET.SubElement(item,'geometry');ET.SubElement(ET.SubElement(geo,'box'),'size').text=f'{sx} {sy} {sz}'
  if tag=='visual':
   mat=ET.SubElement(item,'material');ET.SubElement(mat,'ambient').text='0.65 0.72 0.78 1';ET.SubElement(mat,'diffuse').text='0.65 0.72 0.78 1'
# Ground tiles exclude the ascending ramp footprint, avoiding hidden ground below it.
box('ground_west',-13.5,0,-.02,13,40,.04)
box('ground_east',8,0,-.02,24,40,.04)
box('ground_south',-5.5,-14.5,-.02,3,11,.04)
box('ground_north',-5.5,12.5,-.02,3,15,.04)
# Middle and upper floors have openings for the two ramp flights.
for level in (3,6):
 box(f'floor_{level}',1.5,-2,level-.01,11,18,.02)
 box(f'south_landing_{level}',1.5,-10,level-.01,17,2,.02)
 box(f'north_landing_{level}',1.5,6,level-.01,17,2,.02)
slope=math.atan2(3,14);length=math.hypot(14,3)
box('ramp_ground_to_second',-5.5,-2,1.5-.01,3,length,.02,roll=-slope)
box('ramp_second_to_third',8.5,-2,4.5-.01,3,length,.02,roll=slope)
# Side walls prevent the planner from treating the thin ramp's exposed side
# as a climbable step. Both endpoints remain open for level entry and exit.
for x in (-7., -4.):
 box(f'first_ramp_side_{x}',x,-2,1.5+.35,.12,length,.7,roll=-slope)
for x in (7., 10.):
 box(f'second_ramp_side_{x}',x,-2,4.5+.35,.12,length,.7,roll=slope)
ET.indent(tree,space='  ')
out=root/'src/robot_sim_worlds/worlds/building_accessible.sdf';tree.write(out,encoding='utf-8',xml_declaration=True);print(out)

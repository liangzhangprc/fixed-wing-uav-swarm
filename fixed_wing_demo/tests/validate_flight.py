#!/usr/bin/env python3
"""Validate actual Gazebo telemetry, rather than testing controller formulas."""
import argparse
import csv
import json
import statistics
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('telemetry',type=Path)
p.add_argument('--output',type=Path)
a=p.parse_args()
r=list(csv.DictReader(a.telemetry.open()))
assert r, 'No odometry telemetry recorded'
phases=[x['phase'] for x in r]
for phase in ['TAKEOFF_ROLL','ROTATE','CLIMB','FIGURE_EIGHT']:
    assert phase in phases, f'Missing flight phase: {phase}'
assert max(int(x['lap']) for x in r)>=2, 'Two complete figure-eight laps required'
steady=[x for x in r if x['phase']=='FIGURE_EIGHT' and int(x['lap'])>=2]
alt=[float(x['altitude']) for x in steady]
speed=[float(x['speed']) for x in steady]
error=[float(x['cross_track_error']) for x in steady]
assert min(alt)>50 and max(alt)<70, 'Cruise altitude diverged'
assert min(speed)>13 and max(speed)<24, 'Aircraft airspeed unsafe'
assert statistics.mean(error)<10 and max(error)<35, 'Figure-eight tracking diverged'
flight=[x for x in r if x['phase']=='FIGURE_EIGHT']
assert min(float(x['x']) for x in flight)<1030 and max(float(x['x']) for x in flight)>1370, 'Both horizontal lobes not visited'
assert min(float(x['y']) for x in flight)<-80 and max(float(x['y']) for x in flight)>80, 'Both sides of the figure-eight not visited'
summary={'result':'PASS','completed_laps':max(int(x['lap']) for x in r),
         'simulated_seconds':float(r[-1]['sim_time']),
         'cruise_height_m':[min(alt),max(alt)],'cruise_speed_mps':[min(speed),max(speed)],
         'mean_cross_track_error_m':statistics.mean(error),'max_cross_track_error_m':max(error),
         'takeoff_rotation_time_s':float(next(x['sim_time'] for x in r if x['phase']=='ROTATE')),
         'climb_complete_time_s':float(next(x['sim_time'] for x in r if x['phase']=='FIGURE_EIGHT'))}
print(json.dumps(summary,indent=2))
if a.output:
    a.output.write_text(json.dumps(summary,indent=2)+'\n')

#!/usr/bin/env python3
"""Monitor all aircraft pairs and issue deconflicted mission setpoints."""
import csv
import itertools
import json
import math
from pathlib import Path
import re
import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from swarm_safety import closest_approach,avoidance_targets


class Coordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')
        defaults={'vehicle_count':6,'base_altitude':60.0,'altitude_spacing':18.0,
                  'safety_distance':8.0,'prediction_distance':12.0,'collision_distance':3.0,
                  'prediction_horizon':6.0,'stop_after_one_lap':False,
                  'run_directory':str(Path.home()/'.ros/fixed_wing_demo/swarm')}
        for key,value in defaults.items():self.declare_parameter(key,value)
        self.p={key:self.get_parameter(key).value for key in defaults}
        self.count=self.p['vehicle_count']
        self.base=[self.p['base_altitude']+i*self.p['altitude_spacing'] for i in range(self.count)]
        self.states={};self.phases=['WAITING']*self.count;self.laps=[0]*self.count
        self.outputs=[];self.active=set();self.contacts=set();self.violations=set()
        self.min_distance=math.inf;self.collision_events=0;self.violation_events=0;self.activations=0
        self.last_status=-1.;self.last_log=-1.;self.completed=False
        self.height=list(self.base);self.speed=[18.0]*self.count
        for i in range(self.count):
            prefix=f'/uav_{i+1}'
            self.outputs.append(self.create_publisher(Vector3,prefix+'/avoidance_setpoint',10))
            self.create_subscription(Odometry,prefix+'/odometry',lambda m,i=i:self.odometry(i,m),qos_profile_sensor_data)
            self.create_subscription(String,prefix+'/status',lambda m,i=i:self.mission(i,m),10)
        self.status=self.create_publisher(String,'/swarm/status',10)
        self.directory=Path(self.p['run_directory']);self.directory.mkdir(parents=True,exist_ok=True)
        self.file=(self.directory/'separation.csv').open('w',newline='');self.csv=csv.writer(self.file)
        self.csv.writerow(['sim_time','uav_a','uav_b','distance','predicted_distance','time_to_closest','risk','collision','safety_violation'])
        self.create_timer(0.05,self.update)
        self.get_logger().info(f'{self.count} independent SITL vehicles; checking {self.count*(self.count-1)//2} pairs')

    def odometry(self,i,msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist.linear
        world_v=((1-2*(q.y*q.y+q.z*q.z))*v.x+2*(q.x*q.y-q.w*q.z)*v.y+2*(q.x*q.z+q.w*q.y)*v.z,
                 2*(q.x*q.y+q.w*q.z)*v.x+(1-2*(q.x*q.x+q.z*q.z))*v.y+2*(q.y*q.z-q.w*q.x)*v.z,
                 2*(q.x*q.z-q.w*q.y)*v.x+2*(q.y*q.z+q.w*q.x)*v.y+(1-2*(q.x*q.x+q.y*q.y))*v.z)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.states[i]=((p.x,p.y,p.z),world_v,stamp)

    def mission(self,i,msg):
        self.phases[i]=msg.data.split(' | ')[0]
        lap=re.search(r'lap=(\d+)',msg.data)
        if lap:self.laps[i]=int(lap[1])

    def update(self):
        now=self.get_clock().now().nanoseconds/1e9
        valid=[i for i,s in self.states.items() if now-s[2]<0.5]
        conflicts=set();contacts=set();violations=set();pairs=[]
        for i,j in itertools.combinations(sorted(valid),2):
            a,va,ta=self.states[i];b,vb,tb=self.states[j]
            # Align asynchronously received odometry at the current simulation time.
            a=tuple(x+v*(now-ta) for x,v in zip(a,va));b=tuple(x+v*(now-tb) for x,v in zip(b,vb))
            distance,predicted,t=closest_approach(a,va,b,vb,self.p['prediction_horizon'])
            self.min_distance=min(self.min_distance,distance)
            if distance<self.p['collision_distance']:contacts.add((i,j))
            if distance<self.p['safety_distance']:violations.add((i,j))
            if predicted<self.p['prediction_distance'] or distance<self.p['safety_distance']:
                conflicts.add((i,j))
            # Hysteresis avoids rapid alternation when near the warning boundary.
            elif (i,j) in self.active and predicted<self.p['prediction_distance']+3:
                conflicts.add((i,j))
            pairs.append([now,i+1,j+1,distance,predicted,t,(i,j) in conflicts,(i,j) in contacts,(i,j) in violations])
        new=conflicts-self.active
        self.activations+=len(new);self.collision_events+=len(contacts-self.contacts)
        self.violation_events+=len(violations-self.violations)
        if new:self.get_logger().warn(f'Predicted proximity: {[(i+1,j+1) for i,j in sorted(new)]}; altitude/speed deconfliction active')
        if contacts-self.contacts:self.get_logger().error(f'Collision envelope breached: {sorted(contacts-self.contacts)}')
        self.active=conflicts;self.contacts=contacts;self.violations=violations
        height,speed=avoidance_targets(self.base,conflicts)
        for i,publisher in enumerate(self.outputs):
            # Rate-limit height changes and retain a safe ordered altitude stack.
            self.height[i]+=max(-0.1,min(0.1,height[i]-self.height[i]))
            self.speed[i]+=max(-0.025,min(0.025,speed[i]-self.speed[i]))
            publisher.publish(Vector3(x=self.speed[i],y=0.0,z=self.height[i]))
        if now-self.last_log>=0.2:
            self.last_log=now;self.csv.writerows(pairs);self.file.flush()
        if now-self.last_status>=1:
            self.last_status=now
            report={'sim_time':now,'vehicles_seen':len(valid),'phases':self.phases,'laps':self.laps,
                    'min_distance_m':self.min_distance if math.isfinite(self.min_distance) else None,
                    'collision_events':self.collision_events,'safety_violations':self.violation_events,
                    'avoidance_activations':self.activations,'active_risk_pairs':[[i+1,j+1] for i,j in sorted(conflicts)],
                    'target_altitudes':self.height,'target_speeds':self.speed,
                    'all_completed_one_lap':all(lap>=1 for lap in self.laps)}
            self.status.publish(String(data=json.dumps(report)))
            (self.directory/'swarm_summary.json').write_text(json.dumps(report,indent=2)+'\n')
            if report['all_completed_one_lap'] and not self.completed:
                self.completed=True;self.get_logger().info('All six vehicles completed one figure-eight lap')
                if self.p['stop_after_one_lap']:rclpy.shutdown()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);signal.signal(signal.SIGINT,signal.default_int_handler);node=Coordinator()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.file.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()

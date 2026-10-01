#!/usr/bin/env python3
"""Force-driven fixed-wing mission controller, world ENU / body FLU.
Command Twist: linear.x throttle [0,1], angular.x roll, angular.y pitch.
A positive mathematical roll banks right; nose-up pitch is negative in FLU.
"""
import csv
import math
from pathlib import Path
import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import String


def clamp(x, low, high):
    return max(low, min(high, x))


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


class Autopilot(Node):
    def __init__(self):
        super().__init__('fixed_wing_autopilot')
        defaults = {'altitude': 60.0, 'cruise_speed': 18.0, 'figure_eight_a': 220.0,
                    'figure_eight_b': 110.0, 'center_x': 1200.0, 'lookahead': 35.0,
                    'telemetry_file': str(Path.home()/'.ros/fixed_wing_demo/flight.csv')}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.declare_parameter('swarm_enabled',False)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.target_altitude=self.p['altitude'];self.target_speed=self.p['cruise_speed']
        self.avoidance_time=None
        if self.get_parameter('swarm_enabled').value:
            self.create_subscription(Vector3,'/fixed_wing/avoidance_setpoint',self.avoidance,10)
        self.command = self.create_publisher(Twist, '/fixed_wing/command', 10)
        self.status = self.create_publisher(String, '/fixed_wing/status', 10)
        self.reference = self.create_publisher(RosPath, '/fixed_wing/reference_path', 1)
        self.track = self.create_publisher(RosPath, '/fixed_wing/flown_path', 1)
        self.create_subscription(Odometry, '/fixed_wing/odometry', self.on_odometry, qos_profile_sensor_data)
        self.phase = 'TAKEOFF_ROLL'
        self.index = None
        self.progress = 0
        self.laps = 0
        self.counter = 0
        self.speed_i = 0.0
        self.previous_time = None
        self.last_message_time = self.get_clock().now()
        self.points = [(self.p['center_x']+self.p['figure_eight_a']*math.sin(2*math.pi*i/1600),
                        self.p['figure_eight_b']*math.sin(4*math.pi*i/1600)) for i in range(1600)]
        self.history = RosPath()
        self.history.header.frame_id = 'world'
        file = Path(self.p['telemetry_file']).expanduser()
        file.parent.mkdir(parents=True, exist_ok=True)
        self.file = file.open('w', newline='')
        self.csv = csv.writer(self.file)
        self.csv.writerow(['sim_time', 'phase', 'x', 'y', 'altitude', 'speed', 'roll', 'pitch',
                           'throttle', 'cross_track_error', 'lap', 'path_progress'])
        self.create_timer(0.25, self.watchdog)
        self.get_logger().info('Mission armed; waiting for Gazebo odometry. Telemetry: '+str(file))

    def avoidance(self,msg):
        if math.isfinite(msg.z) and math.isfinite(msg.x):
            self.target_altitude=clamp(msg.z,30.0,200.0)
            self.target_speed=clamp(msg.x,16.0,20.0)
            self.avoidance_time=self.get_clock().now()

    def watchdog(self):
        if self.avoidance_time is not None and (self.get_clock().now()-self.avoidance_time).nanoseconds>1_500_000_000:
            self.target_altitude=self.p['altitude'];self.target_speed=self.p['cruise_speed']
        if (self.get_clock().now()-self.last_message_time).nanoseconds > 1_000_000_000:
            self.command.publish(Twist())

    def transition(self, phase):
        self.phase = phase
        self.get_logger().info('Mission phase: '+phase)

    def on_odometry(self, msg):
        self.last_message_time = self.get_clock().now()
        t = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        dt = clamp(t-(self.previous_time if self.previous_time is not None else t-0.02), 0.001, 0.1)
        self.previous_time = t
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(clamp(2*(q.w*q.y-q.z*q.x), -1, 1))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        b = msg.twist.twist.linear
        # Rotate body velocity into world (full quaternion rotation).
        vx = (1-2*(q.y*q.y+q.z*q.z))*b.x+2*(q.x*q.y-q.w*q.z)*b.y+2*(q.x*q.z+q.w*q.y)*b.z
        vy = 2*(q.x*q.y+q.w*q.z)*b.x+(1-2*(q.x*q.x+q.z*q.z))*b.y+2*(q.y*q.z-q.w*q.x)*b.z
        vz = 2*(q.x*q.z-q.w*q.y)*b.x+2*(q.y*q.z+q.w*q.x)*b.y+(1-2*(q.x*q.x+q.y*q.y))*b.z
        speed = math.sqrt(vx*vx+vy*vy+vz*vz)
        heading = math.atan2(vy,vx) if speed>5 else yaw
        if not all(math.isfinite(v) for v in [pos.x,pos.y,pos.z,speed,roll,pitch]):
            self.command.publish(Twist())
            self.transition('FAULT')
            return
        if self.phase=='TAKEOFF_ROLL' and speed>=13.0:
            self.transition('ROTATE')
        if self.phase=='ROTATE' and pos.z>2.0:
            self.transition('CLIMB')
        if self.phase=='CLIMB' and pos.z>=self.p['altitude']-2.0:
            self.transition('FIGURE_EIGHT')
        rc, pc, throttle, error = 0.0, 0.0, 1.0, 0.0
        if self.phase=='TAKEOFF_ROLL':
            pc = 0.0
        elif self.phase=='ROTATE':
            pc = -0.12
        elif self.phase in ('CLIMB','FIGURE_EIGHT'):
            target_speed = self.target_speed
            self.speed_i = clamp(self.speed_i+(target_speed-speed)*dt, -3.0, 6.0)
            throttle = clamp(0.30+0.10*(target_speed-speed)+0.025*self.speed_i, 0.08, 1.0)
            if self.phase=='FIGURE_EIGHT':
                if self.index is None:
                    # Enter at the central crossing with an eastward tangent.
                    candidates=range(-200,201)
                else:
                    candidates=range(self.index-12,self.index+65)
                idx = min(candidates, key=lambda i:(self.points[i%1600][0]-pos.x)**2+(self.points[i%1600][1]-pos.y)**2)
                if self.index is not None:
                    self.progress += max(0, idx-self.index)
                self.index = idx
                new_laps = self.progress//1600
                if new_laps>self.laps:
                    self.laps=new_laps
                    self.get_logger().info(f'Figure-eight lap {self.laps} complete')
                px, py = self.points[idx%1600]
                error = math.hypot(px-pos.x,py-pos.y)
                aim = idx+1
                distance=0.0
                while distance<self.p['lookahead'] and aim<idx+400:
                    a=self.points[(aim-1)%1600]; z=self.points[aim%1600]
                    distance+=math.hypot(z[0]-a[0],z[1]-a[1])
                    aim+=1
                tx,ty=self.points[(aim-1)%1600]
                desired_heading=math.atan2(ty-pos.y,tx-pos.x)
                eta=wrap(desired_heading-heading)
                look=max(10,math.hypot(tx-pos.x,ty-pos.y))
                lateral_acc=2*max(speed,12)**2*math.sin(clamp(eta,-math.pi/2,math.pi/2))/self.p['lookahead']
                rc=-clamp(math.atan2(lateral_acc,9.81),-0.60,0.60)
            else:
                rc=clamp(0.9*wrap(heading),-0.25,0.25)
            desired_vz=clamp(0.6*(self.target_altitude-pos.z),-2.5,3.0)
            # Lift feed-forward plus vertical-speed feedback yields commanded angle of attack.
            required_cl=3.0*9.81/(max(math.cos(roll),0.65)*0.5*1.225*max(speed,10)**2*0.8)
            alpha_trim=(required_cl-0.10)/4.8
            flight_path_angle=math.atan2(vz,max(math.hypot(vx,vy),8))
            nose_up=flight_path_angle+alpha_trim+0.022*(desired_vz-vz)
            pc=-clamp(nose_up,-0.18,0.25)
        command=Twist()
        command.linear.x=throttle
        command.angular.x=rc
        command.angular.y=pc
        self.command.publish(command)
        self.counter+=1
        if self.counter%10==0:
            self.csv.writerow([t,self.phase,pos.x,pos.y,pos.z,speed,roll,pitch,throttle,error,self.laps,self.progress])
            self.file.flush()
            self.status.publish(String(data=f'{self.phase} | height={pos.z:.1f} m | speed={speed:.1f} m/s | lap={self.laps} | error={error:.1f} m'))
            stamped=PoseStamped(header=msg.header,pose=msg.pose.pose)
            self.history.poses.append(stamped)
            self.history.poses=self.history.poses[-3000:]
            self.history.header.stamp=msg.header.stamp
            self.track.publish(self.history)
        if self.counter%100==0:
            path=RosPath(header=msg.header)
            path.header.frame_id='world'
            for px,py in self.points[::4]+[self.points[0]]:
                p=PoseStamped(header=path.header)
                p.pose.position.x=px; p.pose.position.y=py; p.pose.position.z=self.p['altitude']
                p.pose.orientation.w=1.0
                path.poses.append(p)
            self.reference.publish(path)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler)
    node=Autopilot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if rclpy.ok():node.command.publish(Twist())
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()

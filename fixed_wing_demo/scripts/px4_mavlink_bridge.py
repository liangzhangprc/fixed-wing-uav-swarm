#!/usr/bin/env python3
"""Gazebo simulated sensors -> PX4; native PX4 actuators -> Gazebo.
Simulator TCP 4560 carries HIL_SENSOR/HIL_GPS and HIL_ACTUATOR_CONTROLS.
UDP 14540 carries offboard attitude targets and arming commands.
"""
import csv,math,time,random
from pathlib import Path
import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pymavlink import mavutil

def rotate(q,v):
    x,y,z,w=q.x,q.y,q.z,q.w
    return [(1-2*(y*y+z*z))*v[0]+2*(x*y-w*z)*v[1]+2*(x*z+w*y)*v[2],
            2*(x*y+w*z)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z-w*x)*v[2],
            2*(x*z-w*y)*v[0]+2*(y*z+w*x)*v[1]+(1-2*(x*x+y*y))*v[2]]
def inverse_rotate(q,v):
    from types import SimpleNamespace
    return rotate(SimpleNamespace(x=-q.x,y=-q.y,z=-q.z,w=q.w),v)
def quat(roll,pitch,yaw):
    cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]

class BufferedSimulatorTCP(mavutil.mavtcpin):
    """Preserve complete MAVLink packets across nonblocking partial TCP sends."""
    def __init__(self,port=4560):
        super().__init__(f'127.0.0.1:{port}',source_system=42,source_component=196)
        self.pending=bytearray()
    def write(self,buf):
        if self.port is None:return
        self.pending.extend(buf)
        if len(self.pending)>262144:
            raise RuntimeError('PX4 simulator TCP stalled: send queue exceeded 256 KiB')
        self.flush()
    def flush(self):
        if self.port is None:
            self.pending.clear();return
        while self.pending:
            try:n=self.port.send(self.pending)
            except BlockingIOError:return
            except OSError:
                self.port.close();self.port=None;self.fd=self.listen.fileno()
                self.pending.clear();return
            if not n:return
            del self.pending[:n]

class Bridge(Node):
    def __init__(self):
        super().__init__('px4_mavlink_bridge')
        self.declare_parameter('instance_id',0)
        instance=int(self.get_parameter('instance_id').value)
        self.system_id=instance+1
        self.sim_port=4560+instance;self.offboard_port=14540+instance
        self.rng=random.Random(7+instance)
        mavutil.set_dialect('common')
        self.sim=BufferedSimulatorTCP(self.sim_port)
        self.api=mavutil.mavlink_connection(f'udpin:127.0.0.1:{self.offboard_port}',source_system=42,source_component=191,dialect='common')
        self.actuator=self.create_publisher(Twist,'/fixed_wing/actuator_command',10)
        self.status=self.create_publisher(String,'/px4/sitl_status',10)
        self.create_subscription(Odometry,'/fixed_wing/odometry',self.odometry,qos_profile_sensor_data)
        self.create_subscription(Imu,'/fixed_wing/imu',self.imu,qos_profile_sensor_data)
        self.create_subscription(Twist,'/fixed_wing/command',self.command,10)
        self.odom=None;self.target=Twist();self.armed=False;self.offboard=False;self.have_heartbeat=False
        self.started=time.monotonic();self.last_request=0;self.last_heartbeat=0;self.last_actuator=0
        self.sensor_count=0;self.actuator_count=0;self.last_gps=0;self.last_status=0
        self.declare_parameter('telemetry_file',str(Path.home()/'.ros/fixed_wing_demo/px4_bridge.csv'))
        self.file=Path(self.get_parameter('telemetry_file').value)
        self.file.parent.mkdir(parents=True,exist_ok=True)
        self.log=self.file.open('w',newline='');self.csv=csv.writer(self.log)
        self.csv.writerow(['wall_time','imu_samples','actuator_samples','armed','offboard','roll_output','pitch_output','yaw_output','throttle'])
        self.create_timer(0.005,self.poll)
        self.create_timer(0.02,self.send_target)
        self.get_logger().info(f'PX4 sysid={self.system_id}: TCP {self.sim_port}; UDP {self.offboard_port}')
    def command(self,msg):self.target=msg
    def odometry(self,msg):self.odom=msg
    def imu(self,msg):
        if self.odom is None:return
        p=self.odom.pose.pose.position; q=msg.orientation
        v=self.odom.twist.twist.linear
        mag=inverse_rotate(q,[0.007,0.21,-0.44])
        # FLU -> FRD. HIL_SENSOR uses specific force, not gravity-inclusive inertial acceleration.
        a=msg.linear_acceleration;w=msg.angular_velocity
        stamp=msg.header.stamp.sec*1000000+msg.header.stamp.nanosec//1000
        pressure=1013.25*(1-(488+p.z)/44330)**5.255
        differential=0.5*1.225*max(0,v.x)**2/100
        try:
            self.sim.mav.hil_sensor_send(stamp,a.x+self.rng.gauss(0,0.002),-a.y+self.rng.gauss(0,0.002),-a.z+self.rng.gauss(0,0.002),w.x+self.rng.gauss(0,0.0015),-w.y+self.rng.gauss(0,0.0015),-w.z+self.rng.gauss(0,0.0015),mag[0]+self.rng.gauss(0,0.00001),-mag[1]+self.rng.gauss(0,0.00001),-mag[2]+self.rng.gauss(0,0.00001),pressure+self.rng.gauss(0,0.001),differential,488+p.z,20,8191)
            self.sensor_count+=1
            if stamp-self.last_gps>=100000:
                self.last_gps=stamp
                world_v=rotate(self.odom.pose.pose.orientation,[v.x,v.y,v.z])
                lat=47.397742+p.y/111111.0;lon=8.545594+p.x/(111111.0*math.cos(math.radians(47.397742)))
                speed=math.sqrt(sum(z*z for z in world_v))
                cog=(math.degrees(math.atan2(world_v[0],world_v[1]))%360)*100
                self.sim.mav.hil_gps_send(stamp,3,int(lat*1e7),int(lon*1e7),int((488+p.z)*1000),50,80,int(speed*100),int(world_v[1]*100),int(world_v[0]*100),int(-world_v[2]*100),int(cog),12)
        except (OSError,ValueError) as e:
            self.get_logger().warn(f'Simulator sensor transport: {e}',throttle_duration_sec=5)
    def poll(self):
        self.sim.flush()
        for _ in range(100):
            msg=self.sim.recv_match(blocking=False)
            if msg is None:break
            if msg.get_type()=='HIL_ACTUATOR_CONTROLS':
                self.actuator_count+=1;self.last_actuator=time.monotonic()
                command=Twist()
                if msg.mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    outputs=[float(x) if math.isfinite(float(x)) else 0.0 for x in msg.controls[:4]]
                    command.angular.x=outputs[0];command.angular.y=outputs[1];command.angular.z=outputs[2];command.linear.x=float(max(0.0,min(1.0,outputs[3])))
                self.actuator.publish(command)
                if self.actuator_count%50==0:
                    self.csv.writerow([time.monotonic()-self.started,self.sensor_count,self.actuator_count,self.armed,self.offboard,command.angular.x,command.angular.y,command.angular.z,command.linear.x]);self.log.flush()
        for _ in range(100):
            msg=self.api.recv_match(blocking=False)
            if msg is None:break
            if msg.get_srcSystem()!=self.system_id:continue
            if msg.get_type()=='HEARTBEAT':
                self.have_heartbeat=True
                self.armed=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.offboard=((msg.custom_mode>>16)&0xff)==6
            elif msg.get_type()=='STATUSTEXT':
                self.get_logger().info('PX4: '+msg.text)
            elif msg.get_type()=='COMMAND_ACK':
                self.get_logger().info(f'PX4 command {msg.command} result {msg.result}')
        now=time.monotonic()
        if now-self.last_heartbeat>1:
            self.last_heartbeat=now
            self.api.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
        if self.have_heartbeat and now-self.started>12 and now-self.last_request>3:
            self.last_request=now
            if not self.offboard:
                self.api.mav.command_long_send(self.system_id,1,mavutil.mavlink.MAV_CMD_DO_SET_MODE,0,1,6,0,0,0,0,0)
            elif not self.armed:
                self.api.mav.command_long_send(self.system_id,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
        if now-self.last_status>1:
            self.last_status=now
            self.status.publish(String(data=f'imu={self.sensor_count} actuators={self.actuator_count} armed={self.armed} offboard={self.offboard}'))
        if now-self.last_actuator>1:self.actuator.publish(Twist())
    def send_target(self):
        if not self.have_heartbeat or self.odom is None:return
        q=self.odom.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        targetq=quat(self.target.angular.x,-self.target.angular.y,math.pi/2-yaw)
        self.api.mav.set_attitude_target_send(int((time.monotonic()-self.started)*1000),self.system_id,1,7,targetq,0,0,0,max(0,min(1,self.target.linear.x)))
    def close(self):
        if rclpy.ok():self.actuator.publish(Twist())
        self.sim.close();self.api.close();self.log.close()

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);signal.signal(signal.SIGINT,signal.default_int_handler);node=Bridge()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()

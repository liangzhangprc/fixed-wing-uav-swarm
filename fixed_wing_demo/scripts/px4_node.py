#!/usr/bin/env python3
"""ROS process supervisor for an independent, native PX4 SITL firmware process."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class PX4Node(Node):
    def __init__(self):
        super().__init__('px4_sitl')
        self.declare_parameter('instance_id',0)
        instance=int(self.get_parameter('instance_id').value)
        self.declare_parameter('px4_binary','')
        self.declare_parameter('run_directory',str(Path.home()/'.ros/fixed_wing_demo/px4_run'))
        binary=Path(self.get_parameter('px4_binary').value)
        if not binary.is_file():
            raise RuntimeError(f'PX4 binary does not exist: {binary}. Run build_px4_sitl.sh first.')
        run=Path(self.get_parameter('run_directory').value)
        run.mkdir(parents=True,exist_ok=True)
        share=Path(get_package_share_directory('fixed_wing_demo'))
        (run/'rc.demo').write_text('#!/bin/sh\n. etc/init.d-posix/rcS\n')
        # Each launch starts with clean simulator parameters, preserving previous files.
        for name in ['parameters.bson','parameters_backup.bson']:
            p=run/name
            if p.exists(): p.rename(run/(name+'.previous'))
        env=os.environ.copy()
        env.update(PX4_SYS_AUTOSTART='1030',PX4_SIM_MODEL='none',PX4_SIMULATOR='mavlink')
        rootfs=run/'romfs'
        shutil.copytree(binary.parent.parent/'etc',rootfs,dirs_exist_ok=True,symlinks=False)
        # This board uses MAVLink, so skip optional DDS commands in the runtime copy.
        startup=rootfs/'init.d-posix/rcS'
        lines=startup.read_text().splitlines()
        startup.write_text('\n'.join(
            ': # DDS disabled in fixedwing SITL board' if
            ('UXRCE_DDS_' in line and line.lstrip().startswith('param ')) or
            line.lstrip().startswith('uxrce_dds_client start') else line
            for line in lines)+'\n')
        airframe=rootfs/'init.d-posix/airframes/1030_gazebo-classic_plane'
        settings=(share/'config/px4_rcS').read_text().replace('. etc/init.d-posix/rcS','')
        with airframe.open('a') as f:f.write('\n'+settings+'\n')
        self.process=subprocess.Popen([str(binary),'-d','-i',str(instance),'-w',str(run),str(rootfs),'-s','rc.demo'],
                                      env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                      text=True,start_new_session=True,bufsize=1)
        self.status=self.create_publisher(String,'/px4/process_status',10)
        self.create_timer(1.0,self.poll)
        self.log=(run/'px4-console.log').open('w')
        self.reader=threading.Thread(target=self.read_output,daemon=True)
        self.reader.start()
        self.get_logger().info(f'Native PX4 SITL running as PID {self.process.pid}; log: {run}/px4-console.log')
    def read_output(self):
        for line in self.process.stdout:
            self.log.write(line);self.log.flush()
            print('[PX4] '+line.rstrip(),flush=True)
    def poll(self):
        code=self.process.poll()
        self.status.publish(String(data=f'RUNNING pid={self.process.pid}' if code is None else f'EXITED code={code}'))
        if code is not None:
            raise RuntimeError(f'PX4 exited unexpectedly with code {code}')
    def stop(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try: self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGTERM)
                self.process.wait(timeout=5)
        self.reader.join(timeout=1)
        self.log.close()

def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);signal.signal(signal.SIGINT,signal.default_int_handler); node=PX4Node()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException): pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.stop();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()

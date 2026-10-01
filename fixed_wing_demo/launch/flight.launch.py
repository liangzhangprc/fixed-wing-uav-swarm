"""One command starts the complete, offline fixed-wing mission."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction,TimerAction,SetEnvironmentVariable,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=Path(get_package_share_directory('fixed_wing_demo'))
    prefix=Path(get_package_prefix('fixed_wing_demo'))
    gui=LaunchConfiguration('gui').perform(context).lower()=='true'
    speed=float(LaunchConfiguration('real_time_factor').perform(context))
    # Generate a per-process world to avoid changing installed assets.
    world_dir=Path.home()/'.ros/fixed_wing_demo'
    world_dir.mkdir(parents=True,exist_ok=True)
    world=world_dir/f'world_{os.getpid()}.sdf'
    world.write_text((share/'worlds/airfield.sdf').read_text().replace('@REAL_TIME_FACTOR@',str(speed)))
    cmd=['gz','sim','-r','-v','3','--render-engine-gui',LaunchConfiguration('render_engine').perform(context)]
    if not gui:
        cmd+=['-s']
    cmd+=[str(world)]
    gazebo=ExecuteProcess(cmd=cmd,output='screen')
    actions = [
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('ros_domain_id').perform(context)),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(share/'models')+':'+os.environ.get('GZ_SIM_RESOURCE_PATH','')),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',str(prefix/'lib')+':'+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')),
        SetEnvironmentVariable('GZ_PARTITION',f'fixed_wing_demo_{os.getpid()}'),
        gazebo,
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='fixed_wing_bridge',
             parameters=[{'config_file':str(share/'config/bridge.yaml')}],output='screen'),
        Node(package='fixed_wing_demo',executable='autopilot.py',name='fixed_wing_autopilot',
             parameters=[{'use_sim_time':True,'altitude':float(LaunchConfiguration('altitude').perform(context)),
                          'telemetry_file':LaunchConfiguration('telemetry_file').perform(context)}],output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=gazebo,on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))])),
    ]
    if gui:
        for delay in [7.0,14.0]:
            actions.append(TimerAction(period=delay,actions=[ExecuteProcess(
                cmd=['gz','topic','-t','/gui/track','-m','gz.msgs.CameraTrack','-p',
                     'track_mode: FOLLOW follow_target {name: "fixed_wing"} follow_offset {x: -8 y: -6 z: 4} follow_pgain: 0.12 track_pgain: 0.12'],
                output='log')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('ros_domain_id',default_value='42'),
        DeclareLaunchArgument('render_engine',default_value='ogre'),
        DeclareLaunchArgument('real_time_factor',default_value='1.0'),
        DeclareLaunchArgument('altitude',default_value='60.0'),
        DeclareLaunchArgument('telemetry_file',default_value=str(Path.home()/'.ros/fixed_wing_demo/flight.csv')),
        OpaqueFunction(function=setup),
    ])

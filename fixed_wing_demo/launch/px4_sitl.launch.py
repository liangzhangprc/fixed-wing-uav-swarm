"""Native PX4 firmware + custom Gazebo fixed wing, with a MAVLink SITL loop."""
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
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
    workspace=prefix.parent.parent
    px4_root=Path(LaunchConfiguration('px4_root').perform(context))
    binary=px4_root/'build/px4_sitl_fixedwing/bin/px4'
    venv=workspace/'px4_venv'
    if not binary.is_file():raise RuntimeError(f'Build PX4 SITL first; binary missing: {binary}')
    if not (venv/'bin/python3').exists():raise RuntimeError('PX4 Python environment missing; run build_px4_sitl.sh')
    gui=LaunchConfiguration('gui').perform(context).lower()=='true'
    run=Path.home()/'.ros/fixed_wing_demo'/f'sitl_{os.getpid()}'
    model_dir=run/'models/fixed_wing';model_dir.mkdir(parents=True,exist_ok=True)
    aircraft=ET.parse(share/'models/fixed_wing/model.sdf')
    plugin=aircraft.getroot().find('model/plugin')
    ET.SubElement(plugin,'control_mode').text='actuator'
    aircraft.write(model_dir/'model.sdf',encoding='unicode',xml_declaration=True)
    shutil.copy2(share/'models/fixed_wing/model.config',model_dir/'model.config')
    world=run/'airfield.sdf'
    world.write_text((share/'worlds/airfield.sdf').read_text().replace('@REAL_TIME_FACTOR@','1.0'))
    command=['gz','sim','-r','-v','2','--render-engine-gui','ogre']
    if not gui:command+=['-s']
    command+=[str(world)]
    gazebo=ExecuteProcess(cmd=command,output='screen')
    transport=Node(package='ros_gz_bridge',executable='parameter_bridge',name='px4_gazebo_transport',
                   parameters=[{'config_file':str(share/'config/px4_bridge.yaml')}],output='screen')
    simulator=Node(package='fixed_wing_demo',executable='px4_mavlink_bridge.py',name='px4_mavlink_bridge',parameters=[{'telemetry_file':str(run/'px4_bridge.csv')}],output='screen')
    px4=Node(package='fixed_wing_demo',executable='px4_node.py',name='px4_sitl',
             parameters=[{'px4_binary':str(binary),'run_directory':str(run/'px4')}],output='screen')
    mission=Node(package='fixed_wing_demo',executable='autopilot.py',name='px4_mission',
                 parameters=[{'use_sim_time':True,'telemetry_file':str(run/'flight.csv')}],output='screen')
    actions=[
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('ros_domain_id').perform(context)),
        SetEnvironmentVariable('MAVLINK20','1'),
        SetEnvironmentVariable('PYTHONFAULTHANDLER','1'),
        SetEnvironmentVariable('PATH',str(venv/'bin')+':'+os.environ.get('PATH','')),
        SetEnvironmentVariable('GZ_PARTITION',f'px4_sitl_{os.getpid()}'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(run/'models')+':'+str(share/'models')+':'+os.environ.get('GZ_SIM_RESOURCE_PATH','')),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',str(prefix/'lib')+':'+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')),
        gazebo,transport,simulator,mission,
        TimerAction(period=2.0,actions=[px4]),
    ]
    for action in [gazebo,transport,simulator,px4,mission]:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=action,on_exit=[EmitEvent(event=Shutdown(reason='A required SITL process exited'))])))
    if gui:
        for delay in [7.0,14.0]:
            actions.append(TimerAction(period=delay,actions=[ExecuteProcess(
                cmd=['gz','topic','-t','/gui/track','-m','gz.msgs.CameraTrack','-p',
                     'track_mode: FOLLOW follow_target {name: "fixed_wing"} follow_offset {x: -8 y: -6 z: 4} follow_pgain: 0.12 track_pgain: 0.12'],output='log')]))
    return actions


def generate_launch_description():
    workspace=Path(get_package_prefix('fixed_wing_demo')).parent.parent
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true',description='Open Gazebo GUI'),
        DeclareLaunchArgument('ros_domain_id',default_value='43',description='ROS domain for SITL nodes'),
        DeclareLaunchArgument('px4_root',default_value=str(workspace/'src/PX4-Autopilot'),description='PX4 source tree with compiled fixedwing SITL'),
        OpaqueFunction(function=setup),
    ])

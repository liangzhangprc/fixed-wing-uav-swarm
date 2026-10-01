"""Six independent PX4 fixed-wing SITL instances and a shared deconflicted path."""
import copy
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction,TimerAction,SetEnvironmentVariable,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share=Path(get_package_share_directory('fixed_wing_demo'))
    prefix=Path(get_package_prefix('fixed_wing_demo'));workspace=prefix.parent.parent
    px4_root=Path(LaunchConfiguration('px4_root').perform(context))
    binary=px4_root/'build/px4_sitl_fixedwing/bin/px4';venv=workspace/'px4_venv'
    if not binary.is_file() or not (venv/'bin/python3').is_file():
        raise RuntimeError('PX4 binary or Python environment missing; run build_px4_sitl.sh')
    gui=LaunchConfiguration('gui').perform(context).lower()=='true'
    stop=LaunchConfiguration('stop_after_one_lap').perform(context).lower()=='true'
    count=6;spacing=18.0
    run=Path.home()/'.ros/fixed_wing_demo'/f'swarm_{os.getpid()}'
    run.mkdir(parents=True,exist_ok=True)
    tree=ET.ElementTree(ET.fromstring((share/'worlds/airfield.sdf').read_text().replace('@REAL_TIME_FACTOR@','1.0')))
    world=tree.getroot().find('world')
    for element in list(world.findall('include')):world.remove(element)
    scene=world.find("gui/plugin[@filename='MinimalScene']/camera_pose")
    scene.text='-170 -350 210 0 0.47 1.05'
    tracking=world.find("gui/plugin[@filename='CameraTracking']")
    tracking.find('follow_target').text='uav_3'
    tracking.find('follow_offset').text='-220 -380 200'
    terrain=world.find("model[@name='terrain']/link")
    # Six clearly marked parallel takeoff lanes on the original flat airfield.
    visuals=[v for v in terrain.findall('visual') if v.get('name')=='runway' or v.get('name','').startswith('centerline_')]
    for v in visuals:terrain.remove(v)
    bridges=[yaml.safe_load((share/'config/px4_bridge.yaml').read_text())[0]]
    template=yaml.safe_load((share/'config/px4_bridge.yaml').read_text())[1:]
    colors=['0.95 0.25 0.15 1','0.95 0.65 0.08 1','0.20 0.85 0.35 1',
            '0.10 0.55 0.95 1','0.75 0.25 0.95 1','0.95 0.95 0.95 1']
    children=[];immediate=[];supervisors=[];manifest=[]
    for i in range(count):
        name=f'uav_{i+1}';ns='/'+name;y=(i-2.5)*spacing
        model_dir=run/'models'/name;model_dir.mkdir(parents=True,exist_ok=True)
        aircraft=ET.parse(share/'models/fixed_wing/model.sdf')
        model=aircraft.getroot().find('model');model.set('name',name)
        plugin=model.find('plugin')
        ET.SubElement(plugin,'control_mode').text='actuator';ET.SubElement(plugin,'topic_prefix').text=ns
        for material in model.findall('.//visual/material'):
            for field in ['ambient','diffuse']:
                child=material.find(field)
                if child is not None:child.text=colors[i]
        aircraft.write(model_dir/'model.sdf',encoding='unicode',xml_declaration=True)
        (model_dir/'model.config').write_text(f'<model><name>{name}</name><version>1.0</version><sdf version="1.10">model.sdf</sdf></model>')
        include=ET.SubElement(world,'include');ET.SubElement(include,'uri').text='model://'+name
        ET.SubElement(include,'name').text=name;ET.SubElement(include,'pose').text=f'0 {y} 0.32 0 0 0'
        for v in visuals:
            lane=copy.deepcopy(v);lane.set('name',name+'_'+v.get('name'))
            pose=lane.find('pose');numbers=pose.text.split();numbers[1]=str(float(numbers[1])+y);pose.text=' '.join(numbers)
            terrain.append(lane)
        for item in template:
            bridges.append({key:value.replace('/fixed_wing',ns) if isinstance(value,str) else value for key,value in item.items()})
        topics=['odometry','imu','command','actuator_command','avoidance_setpoint','status','reference_path','flown_path']
        remap=[('/fixed_wing/'+topic,ns+'/'+topic) for topic in topics]
        remap += [('/px4/sitl_status',ns+'/px4/sitl_status'),('/px4/process_status',ns+'/px4/process_status')]
        vehicle_run=run/name;vehicle_run.mkdir(exist_ok=True)
        simulator=Node(package='fixed_wing_demo',executable='px4_mavlink_bridge.py',namespace=name,name='mavlink_bridge',
                       parameters=[{'instance_id':i,'telemetry_file':str(vehicle_run/'px4_bridge.csv')}],remappings=remap,output='screen')
        mission=Node(package='fixed_wing_demo',executable='autopilot.py',namespace=name,name='mission',
                     parameters=[{'use_sim_time':True,'altitude':60.0+i*spacing,'swarm_enabled':True,
                                  'telemetry_file':str(vehicle_run/'flight.csv')}],remappings=remap,output='screen')
        px4=Node(package='fixed_wing_demo',executable='px4_node.py',namespace=name,name='px4_sitl',
                 parameters=[{'instance_id':i,'px4_binary':str(binary),'run_directory':str(vehicle_run/'px4')}],remappings=remap,output='screen')
        children.extend([simulator,mission]);immediate.extend([simulator,mission]);supervisors.append(TimerAction(period=2.0+i*0.25,actions=[px4]))
        children.append(px4)
        manifest.append({'name':name,'px4_instance':i,'mavlink_system_id':i+1,'simulator_tcp':4560+i,
                         'offboard_udp':14540+i,'altitude_m':60.0+i*spacing,'spawn_y_m':y})
    tree.write(run/'airfield.sdf',encoding='unicode',xml_declaration=True)
    (run/'bridge.yaml').write_text(yaml.safe_dump(bridges))
    (run/'vehicles.json').write_text(json.dumps(manifest,indent=2)+'\n')
    command=['gz','sim','-r','-v','2','--render-engine-gui','ogre']
    if not gui:command.append('-s')
    gazebo=ExecuteProcess(cmd=command+[str(run/'airfield.sdf')],output='screen')
    transport=Node(package='ros_gz_bridge',executable='parameter_bridge',name='swarm_gazebo_transport',
                   parameters=[{'config_file':str(run/'bridge.yaml')}],output='screen')
    coordinator=Node(package='fixed_wing_demo',executable='swarm_coordinator.py',name='swarm_coordinator',
                     parameters=[{'use_sim_time':True,'run_directory':str(run),'stop_after_one_lap':stop}],output='screen')
    actions=[SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('ros_domain_id').perform(context)),
             SetEnvironmentVariable('MAVLINK20','1'),SetEnvironmentVariable('PYTHONFAULTHANDLER','1'),
             SetEnvironmentVariable('PATH',str(venv/'bin')+':'+os.environ.get('PATH','')),
             SetEnvironmentVariable('GZ_PARTITION',f'px4_swarm_{os.getpid()}'),
             SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(run/'models')+':'+str(share/'models')+':'+os.environ.get('GZ_SIM_RESOURCE_PATH','')),
             SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',str(prefix/'lib')+':'+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')),
             gazebo,transport,coordinator]
    # PX4 needs its simulator listeners to be bound before firmware starts.
    actions += immediate+supervisors
    for child in [gazebo,transport,coordinator]+children:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=child,on_exit=[EmitEvent(event=Shutdown(reason='A required swarm process exited'))])))
    if gui:
        for delay in [7.0,14.0]:
            actions.append(TimerAction(period=delay,actions=[ExecuteProcess(cmd=['gz','topic','-t','/gui/track','-m','gz.msgs.CameraTrack','-p',
                'track_mode: FOLLOW follow_target {name: "uav_3"} follow_offset {x: -220 y: -380 z: 200} follow_pgain: 0.12 track_pgain: 0.12'],output='log')]))
    return actions


def generate_launch_description():
    workspace=Path(get_package_prefix('fixed_wing_demo')).parent.parent
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true',description='Open Gazebo visualization'),
        DeclareLaunchArgument('ros_domain_id',default_value='44',description='ROS domain for six-vehicle swarm'),
        DeclareLaunchArgument('stop_after_one_lap',default_value='false',description='Validation: exit after all six complete one lap'),
        DeclareLaunchArgument('px4_root',default_value=str(workspace/'src/PX4-Autopilot')),
        OpaqueFunction(function=setup)])

#!/usr/bin/env python3
"""Repeat complete delivery cold starts with the CURRENT scene limits unchanged."""
import argparse,hashlib,json,os,shlex,shutil,signal,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]


def runtime_check(folder,nav,tasks):
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import GetParameters
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('repeat_flow_config_probe');proof={}
    follow=nav['controller_server']['ros__parameters']['FollowPath'];planner=nav['planner_server']['ros__parameters']['GridBased'];smooth=nav['velocity_smoother']['ros__parameters'];navigator=tasks['simple_nav2_navigator']['ros__parameters']
    checks={'/controller_server':{'FollowPath.'+k:follow[k] for k in ['desired_linear_vel','rotate_to_heading_angular_vel','max_angular_accel','linear_acceleration','linear_deceleration','max_angular_velocity','angular_deceleration']},'/planner_server':{'GridBased.'+k:planner[k] for k in ['nominal_speed','linear_acceleration','linear_deceleration','max_angular_velocity','angular_acceleration','angular_deceleration']},'/velocity_smoother':{k:smooth[k] for k in ['max_velocity','min_velocity','max_accel','max_decel']},'/simple_nav2_navigator':{k:navigator[k] for k in ['crossing_cruise_speed','crossing_acceleration']}}
    def query(target,names):
        client=node.create_client(GetParameters,target+'/get_parameters');assert client.wait_for_service(timeout_sec=20),target
        try:
            for _ in range(3):
                future=client.call_async(GetParameters.Request(names=names));rclpy.spin_until_future_complete(node,future,timeout_sec=10)
                if future.done() and future.result():return future.result().values
            raise RuntimeError('Parameter response missing: '+target)
        finally:node.destroy_client(client)
    try:
        for target,values in checks.items():
            for (key,expected),value in zip(values.items(),query(target,list(values))):
                actual=list(value.double_array_value) if value.type==8 else value.double_value
                assert actual==expected,(target,key,actual,expected);proof[target+'.'+key]=actual
        initial_tree=query('/simple_nav2_navigator',['initial_plan_bt_xml'])[0].string_value
        assert initial_tree==str(ROOT/'tools/continuous_navigation.xml'),initial_tree
        libraries=list(query('/bt_navigator',['plugin_lib_names'])[0].string_array_value)
        assert 'aic_current_goal_navigation_bt_node' in libraries
        proof['initial_plan_bt_xml']=initial_tree;proof['bt_plugin_lib_names']=libraries
        desc=query('/robot_state_publisher',['robot_description'])[0].string_value
        (folder/'live-robot.urdf').write_text(desc);robot=ET.fromstring(desc);plugin=robot.find("gazebo/plugin[@name='body_force_controller']")
        assert plugin is not None and robot.find("gazebo/plugin[@name='differential_drive_controller']") is None
        assert float(plugin.findtext('max_linear_speed'))==smooth['max_velocity'][0]
        assert float(plugin.findtext('max_angular_speed'))==smooth['max_velocity'][2]
        expected=ET.parse(ROOT/'src/yzbot/mybot_description/urdf/originbot_with_arm_gazebo.xacro').getroot().find(".//plugin[@name='body_force_controller']")
        for key in ['forward_limit','lateral_limit','yaw_limit']:
            actual=float(plugin.findtext(key));assert actual==float(expected.findtext(key)),(key,actual,expected.findtext(key))
            proof['body_force.'+key]=actual
        for side in ['left','right']:
            for key in ['mu1','mu2']:assert float(robot.findtext(f"gazebo[@reference='{side}_wheel_link']/{key}"))==1
        header=json.loads((folder/'native.jsonl').open().readline())
        assert header['left_motor_fmax']==header['right_motor_fmax']==0
        assert header['actual_contact_max_correcting_vel']==200 and abs(header['max_step_size']-.001)<1e-10
        proof['native']=header;(folder/'runtime-verification.json').write_text(json.dumps(proof,indent=2))
    finally:node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--windows-output',type=Path,required=True);parser.add_argument('--repeats',type=int,default=2);args=parser.parse_args()
    if args.repeats<1:raise ValueError('Repeats must be positive')
    out=args.output.expanduser().resolve();destination=args.windows_output.expanduser().resolve()
    if out.exists():raise RuntimeError('Output exists; choose a fresh directory')
    if subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode==0 or json.loads((ROOT/'log/run/processes.json').read_text()):raise RuntimeError('A simulation is active')
    out.mkdir(parents=True);destination.mkdir(parents=True,exist_ok=True)
    scene=ROOT/'scenarios/short_routes_20261007';nav=yaml.safe_load((scene/'nav2.yaml').read_text());tasks=yaml.safe_load((scene/'tasks.yaml').read_text())
    body=ET.parse(ROOT/'src/yzbot/mybot_description/urdf/originbot_with_arm_gazebo.xacro').getroot().find(".//plugin[@name='body_force_controller']")
    force_limits={key:float(body.findtext(key)) for key in ['forward_limit','lateral_limit','yaw_limit']}
    (out/'protocol.json').write_text(json.dumps(dict(force_limits=force_limits,repeats=args.repeats,linear_speed=nav['velocity_smoother']['ros__parameters']['max_velocity'][0],angular_speed=nav['velocity_smoother']['ros__parameters']['max_velocity'][2],acceleration=nav['velocity_smoother']['ros__parameters']['max_accel'],deceleration=nav['velocity_smoother']['ros__parameters']['max_decel'],method='Independent cold starts; current configuration, no inter-trial tuning; headless; task events and raw poll timings retained'),indent=2))
    sources=[HERE/'repeat_flow.py',HERE/'native_probe.cpp',ROOT/'tools/benchmark_delivery.py',ROOT/'tools/record_navigation.py',ROOT/'src/aic_body_drive/src/body_drive.cpp',ROOT/'src/yzbot/mybot_description/urdf/originbot_with_arm_gazebo.xacro',ROOT/'src/nav_simple/nav_simple/simple_navigator.py']+list((ROOT/'src/aic_timed_planner/src').glob('*.cpp'))+list((ROOT/'src/aic_timed_planner/include').rglob('*.hpp'))
    sources += list((ROOT/'src/main_controller/main_controller').glob('*.py'))
    sources += [ROOT/'tools/continuous_navigation.xml',ROOT/'tools/project_launcher.py']
    for source in sources:
        target=out/'source-snapshot'/source.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    binaries=[ROOT/'install/aic_timed_planner/lib/libaic_timed_planner.so',ROOT/'install/aic_body_drive/lib/libaic_body_drive.so',ROOT/'install/aic_timed_planner/lib/libaic_current_goal_navigation_bt_node.so']
    hashes={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources+binaries};(out/'source-binary-hashes.json').write_text(json.dumps(hashes,indent=2))
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','gazebo'],text=True));subprocess.run(['g++','-shared','-fPIC','-std=c++17',str(HERE/'native_probe.cpp'),'-o',str(out/'libnative_probe.so'),*flags],check=True)
    env=os.environ.copy()
    for key in ['AIC_WORLD','AIC_MAP','AIC_NAV_PARAMS','AIC_TASK_CONFIG','AIC_BODY_MAX_LINEAR_SPEED','AIC_BODY_MAX_ANGULAR_SPEED']:env.pop(key,None)
    env.update(AIC_DRIVE_MODEL='body_force',FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'tools/fastdds_wsl_udp.xml'),PYTHONNOUSERSITE='1');os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=env['FASTRTPS_DEFAULT_PROFILES_FILE']
    def launch(action,folder,flow_env):
        with (folder/(action+'.log')).open('w') as log:return subprocess.run(['bash',str(ROOT/'ros_competition_start.sh'),action,'--headless'],cwd=ROOT,env=flow_env,stdout=log,stderr=subprocess.STDOUT,timeout=240)
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted);statuses=[]
    for index in range(1,args.repeats+1):
        folder=out/f'trial-{index}';folder.mkdir()
        for name in ['nav2.yaml','tasks.yaml']:shutil.copy2(scene/name,folder/name)
        tree=ET.parse(scene/'office_test.world');plugin=ET.SubElement(tree.find('world'),'plugin',name='speed_audit_native',filename=str(out/'libnative_probe.so'));ET.SubElement(plugin,'output_file').text=str(folder/'native.jsonl');ET.SubElement(plugin,'finite_rotation').text='false';tree.write(folder/'runtime.world',encoding='utf-8',xml_declaration=True)
        flow_env=dict(env,AIC_WORLD=str(folder/'runtime.world'),AIC_MAP=str(scene/'mapn3.yaml'),AIC_NAV_PARAMS=str(folder/'nav2.yaml'),AIC_TASK_CONFIG=str(folder/'tasks.yaml'))
        print('TRIAL_BEGIN',index,flush=True);recorder=None;stream=None
        try:
            launch('start',folder,flow_env).check_returncode();runtime_check(folder,nav,tasks)
            stream=(folder/'recorder.log').open('w');recorder=subprocess.Popen([sys.executable,str(ROOT/'tools/record_navigation.py'),'--output',str(folder/'observed')],cwd=ROOT,env=flow_env,stdout=stream,stderr=subprocess.STDOUT)
            with (folder/'benchmark-console.log').open('w') as log:
                run=subprocess.run(['bash',str(ROOT/'ros_competition_start.sh'),'benchmark','--headless','--use-running','--log-dir',str(folder/'round-1')],cwd=ROOT,env=flow_env,stdout=log,stderr=subprocess.STDOUT,timeout=480)
            result_path=folder/'round-1/result.json';result=json.loads(result_path.read_text()) if result_path.exists() else {}
            status=dict(trial=index,returncode=run.returncode,**{k:result.get(k) for k in ['complete','grasp_count','place_count','all_five_on_zone_surface','task_event_wall_seconds','task_event_sim_seconds','wall_seconds','sim_seconds','aborted_for_collision','aborted_for_instability']})
            if not result:status['error']='No benchmark result; inspect console'
        except Exception as exc:
            status=dict(trial=index,error=repr(exc));(folder/'runner-error.json').write_text(json.dumps(status,indent=2))
        finally:
            if recorder is not None:recorder.send_signal(signal.SIGINT);recorder.wait(timeout=20);stream.close()
            if (folder/'round-1').exists():
                for log in (folder/'round-1').glob('*.log'):
                    if log.is_symlink():content=log.read_bytes();log.unlink();log.write_bytes(content)
            shutil.copytree(ROOT/'log/run',folder/'stack-logs',dirs_exist_ok=True);launch('stop',folder,flow_env).check_returncode()
        statuses.append(status);(out/'run-status.json').write_text(json.dumps(statuses,indent=2));print('TRIAL_END',json.dumps(status),flush=True)
    assert json.loads((ROOT/'log/run/processes.json').read_text())=={}
    print('ALL ATTEMPTS FINISHED; stack stopped',flush=True)
    if any((out/f'trial-{index}/round-1/result.json').exists() for index in range(1,args.repeats+1)):
        subprocess.run([sys.executable,str(HERE/'analyze_repeats.py'),'--output',str(out),'--windows-output',str(destination)],check=True)


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Run 3/4/5 m/s body-force short trials and one five-cargo flow per speed."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import shlex
import sys
import xml.etree.ElementTree as ET
import yaml

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def runtime_check(folder,speed):
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('speed_ladder_runtime_check');proof={}
    checks={'/controller_server':{'FollowPath.desired_linear_vel':speed},'/velocity_smoother':{'max_velocity':[speed,0.,4.],'max_accel':[3.5,0.,10.],'max_decel':[-5.,0.,-10.]},'/planner_server':{'GridBased.nominal_speed':speed,'GridBased.slow_near_static_obstacles':False},'/simple_nav2_navigator':{'crossing_cruise_speed':speed}}
    try:
        for target,values in checks.items():
            c=node.create_client(GetParameters,target+'/get_parameters')
            assert c.wait_for_service(timeout_sec=20),target
            for attempt in range(3):
                f=c.call_async(GetParameters.Request(names=list(values)))
                rclpy.spin_until_future_complete(node,f,timeout_sec=10)
                if f.done() and f.result():break
            assert f.done() and f.result(),target
            for (key,expected),v in zip(values.items(),f.result().values):
                actual={1:v.bool_value,3:v.double_value,8:list(v.double_array_value)}.get(v.type)
                assert actual==expected,(target,key,actual,expected)
                proof[target+'.'+key]=actual
            node.destroy_client(c)
        c=node.create_client(GetParameters,'/robot_state_publisher/get_parameters');assert c.wait_for_service(timeout_sec=20)
        f=c.call_async(GetParameters.Request(names=['robot_description']));rclpy.spin_until_future_complete(node,f,timeout_sec=20)
        assert f.done() and f.result()
        robot=f.result().values[0].string_value;(folder/'live-robot.urdf').write_text(robot);xml=ET.fromstring(robot)
        assert xml.find("gazebo/plugin[@name='differential_drive_controller']") is None
        body=xml.find("gazebo/plugin[@name='body_force_controller']")
        assert float(body.findtext('max_linear_speed'))==5
        for key,expected in [('forward_limit',150),('lateral_limit',200),('yaw_limit',100)]:assert float(body.findtext(key))==expected
        native=json.loads((folder/'native.jsonl').open().readline())
        assert native['actual_contact_max_correcting_vel']==200 and native['left_motor_fmax']==native['right_motor_fmax']==0
        proof['native']=native
        (folder/'runtime-verification.json').write_text(json.dumps(proof,indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--windows-output',type=Path,required=True)
    choice=parser.add_mutually_exclusive_group()
    choice.add_argument('--short-only',action='store_true')
    choice.add_argument('--flows-only',action='store_true')
    args=parser.parse_args();out=args.output.expanduser().resolve()
    if out.exists():raise RuntimeError('Choose a fresh output directory; no records will be replaced')
    state=ROOT/'log/run/processes.json'
    if (state.exists() and json.loads(state.read_text())) or subprocess.run(['pgrep','-x','gzserver'],stdout=subprocess.DEVNULL).returncode==0:
        raise RuntimeError('Another simulation is active')
    out.mkdir(parents=True);args.windows_output.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy()
    env.update(PYTHONNOUSERSITE='1',AIC_DRIVE_MODEL='body_force',AIC_BODY_MAX_LINEAR_SPEED='5',AIC_BODY_MAX_ANGULAR_SPEED='4',FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'tools/fastdds_wsl_udp.xml'))
    # Parent-side rclpy parameter probes must use the same DDS transport as children.
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=env['FASTRTPS_DEFAULT_PROFILES_FILE']
    order=[] if args.flows_only else [3.,4.,5.,5.,4.,3.]
    protocol=dict(speeds=[3.,4.,5.],short_order=order,short_hold=3.2,short_brake=2.2,omega=2.,body_speed_limit=5.,full_flow_speeds=[] if args.short_only else [3.,4.,5.],method=('One complete-delivery cold start per speed; no short trials' if args.flows_only else 'Counterbalanced short cold starts; optional delivery cold starts per speed')+'; fixed servo/physics/acceleration')
    (out/'ladder-protocol.json').write_text(json.dumps(protocol,indent=2))
    snapshot=out/'source-snapshot';snapshot.mkdir()
    for file in list(HERE.glob('*.py'))+[HERE/'native_probe.cpp',HERE/'empty-field.world',ROOT/'src/aic_body_drive/src/body_drive.cpp',ROOT/'src/yzbot/mybot_description/urdf/originbot_with_arm_gazebo.xacro',ROOT/'tools/record_navigation.py',ROOT/'tools/benchmark_delivery.py']:
        shutil.copy2(file,snapshot/file.name)
    (out/'source-hashes.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in snapshot.iterdir()},indent=2))
    results=[];counts={v:0 for v in order};native_library=out/'libnative_probe.so'
    flags=shlex.split(subprocess.check_output(['pkg-config','--cflags','--libs','gazebo'],text=True))
    subprocess.run(['g++','-shared','-fPIC','-std=c++17',str(HERE/'native_probe.cpp'),'-o',str(native_library),*flags],check=True)
    def launch(action,folder,flow_env):
        with (folder/(action+'.log')).open('w') as log:
            return subprocess.run(['bash',str(ROOT/'ros_competition_start.sh'),action,'--headless'],cwd=ROOT,env=flow_env,stdout=log,stderr=subprocess.STDOUT,timeout=240)
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    for speed in order:
        counts[speed]+=1;folder=out/f'short-{speed:g}-r{counts[speed]}'
        print('SHORT_BEGIN',speed,counts[speed],flush=True)
        with (out/(folder.name+'.log')).open('w') as log:
            run=subprocess.run([sys.executable,str(HERE/'run.py'),'--suite','drive','--variants','body_force','--repeats','1','--speed',str(speed),'--omega','2','--body-speed-limit','5','--output',str(folder)],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
        rows=json.loads((folder/'individual-metrics.json').read_text()) if (folder/'individual-metrics.json').exists() else []
        results.append(dict(kind='short',speed=speed,repeat=counts[speed],folder=str(folder),returncode=run.returncode,input_valid_motions=len(rows)))
        if (folder/'libnative_probe.so').exists():native_library=folder/'libnative_probe.so'
        (out/'run-status.json').write_text(json.dumps(results,indent=2))
        print('SHORT_END',json.dumps(results[-1]),flush=True)
    if not args.short_only:
        for speed in protocol['full_flow_speeds']:
            folder=out/f'flow-{speed:g}';folder.mkdir()
            nav=yaml.safe_load((ROOT/'scenarios/short_routes_20261007/nav2.yaml').read_text())
            nav['controller_server']['ros__parameters']['FollowPath'].update(desired_linear_vel=speed,rotate_to_heading_angular_vel=4.,max_angular_accel=10.,linear_acceleration=3.5,linear_deceleration=5.,max_angular_velocity=4.,angular_deceleration=10.)
            nav['velocity_smoother']['ros__parameters'].update(max_velocity=[speed,0.,4.],max_accel=[3.5,0.,10.],max_decel=[-5.,0.,-10.])
            nav['velocity_smoother']['ros__parameters']['min_velocity'][2]=-4.
            nav['planner_server']['ros__parameters']['GridBased'].update(nominal_speed=speed,linear_acceleration=3.5,linear_deceleration=5.,angular_acceleration=10.,max_angular_velocity=4.,angular_deceleration=10.)
            tasks=yaml.safe_load((ROOT/'scenarios/short_routes_20261007/tasks.yaml').read_text())
            tasks['simple_nav2_navigator']['ros__parameters'].update(crossing_cruise_speed=speed,crossing_acceleration=3.5)
            (folder/'nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False));(folder/'tasks.yaml').write_text(yaml.safe_dump(tasks,sort_keys=False))
            tree=ET.parse(ROOT/'scenarios/short_routes_20261007/office_test.world')
            plugin=ET.SubElement(tree.find('world'),'plugin',name='speed_audit_native',filename=str(native_library))
            ET.SubElement(plugin,'output_file').text=str(folder/'native.jsonl');ET.SubElement(plugin,'finite_rotation').text='false'
            tree.write(folder/'runtime.world',encoding='utf-8',xml_declaration=True)
            flow_env=dict(env,AIC_WORLD=str(folder/'runtime.world'),AIC_MAP=str(ROOT/'scenarios/short_routes_20261007/mapn3.yaml'),AIC_NAV_PARAMS=str(folder/'nav2.yaml'),AIC_TASK_CONFIG=str(folder/'tasks.yaml'))
            print('FLOW_BEGIN',speed,flush=True)
            recorder=None;stream=None
            try:
                launch('start',folder,flow_env).check_returncode()
                runtime_check(folder,speed)
                stream=(folder/'recorder.log').open('w')
                recorder=subprocess.Popen([sys.executable,str(ROOT/'tools/record_navigation.py'),'--output',str(folder/'observed')],env=flow_env,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
                with (folder/'benchmark-console.log').open('w') as log:
                    run=subprocess.run(['bash',str(ROOT/'ros_competition_start.sh'),'benchmark','--headless','--use-running','--log-dir',str(folder/'round-1')],cwd=ROOT,env=flow_env,stdout=log,stderr=subprocess.STDOUT,timeout=480)
                result_path=folder/'round-1/result.json'
                data=json.loads(result_path.read_text()) if result_path.exists() else {}
                results.append(dict(kind='flow',speed=speed,folder=str(folder),returncode=run.returncode,**{k:data.get(k) for k in ('complete','task_event_wall_seconds','task_event_sim_seconds','all_five_in_zones','all_five_on_zone_surface','aborted_for_collision','aborted_for_instability')}))
                if result_path.exists():
                    subprocess.run([sys.executable,str(ROOT/'tools/plot_delivery_motion.py'),'--run-dir',str(folder/'round-1'),'--output',str(args.windows_output/f'delivery-speed-{speed:g}.png'),'--drive-label',f'车体施力 {speed:g} m/s'],check=True)
            except Exception as exc:
                results.append(dict(kind='flow',speed=speed,folder=str(folder),error=repr(exc)))
                (folder/'runner-error.json').write_text(json.dumps(dict(error=repr(exc)),indent=2))
            finally:
                if recorder is not None:
                    recorder.send_signal(signal.SIGINT);recorder.wait(timeout=20);stream.close()
                if (folder/'round-1').exists():
                    for log in (folder/'round-1').glob('*.log'):
                        if log.is_symlink():content=log.read_bytes();log.unlink();log.write_bytes(content)
                shutil.copytree(ROOT/'log/run',folder/'stack-logs',dirs_exist_ok=True)
                launch('stop',folder,flow_env).check_returncode()
            (out/'run-status.json').write_text(json.dumps(results,indent=2))
            print('FLOW_END',json.dumps(results[-1]),flush=True)
    print('ALL SPEED ATTEMPTS FINISHED; stack stopped; inspect each result for completion',flush=True)
    if any((out/f'flow-{speed:g}/round-1/result.json').exists() for speed in protocol['full_flow_speeds']):
        subprocess.run([sys.executable,str(HERE/'analyze_flow_ladder.py'),'--output',str(out),'--windows-output',str(args.windows_output)],check=True)


if __name__=='__main__':main()

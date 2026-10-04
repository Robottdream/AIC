"""Check world-to-pedestal angles and the gate before bending the arm."""
import importlib.util
import math
import time
from pathlib import Path
from types import SimpleNamespace
path=Path(__file__).resolve().parents[1]/'src/arm_action_integration/arm_action_integration/arm_grab_place_node.py'
spec=importlib.util.spec_from_file_location('arm_alignment',path); module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
C=module.ArmGrabPlaceNode
now=time.monotonic()
fake=SimpleNamespace(world_target={'x':0.,'y':1.,'kind':'grab'},base_pose=(0,0,0),base_received=now,yaw_position=0.,yaw_received=now)
assert abs(C.desired_yaw(fake)-math.pi/2)<1e-10
fake.base_pose=(0,0,math.pi/2); assert abs(C.desired_yaw(fake))<1e-10
fake.base_pose=(0,0,-math.pi/2); assert abs(abs(C.desired_yaw(fake))-math.pi)<1e-10
fake.world_target={'x':-1.,'y':0.,'kind':'grab'}; fake.base_pose=(0,0,0)
assert abs(abs(C.desired_yaw(fake))-math.pi)<1e-10
fake.base_received=now-1.
try: C.desired_yaw(fake)
except RuntimeError: pass
else: raise AssertionError('Stale pose used to bend')
fake.base_received=now; fake.world_target={'x':0.,'y':1.,'kind':'grab'}
fake.align_enabled=True; fake.current_task='grab'; fake.yaw_busy=False
fake.arm_trajectory={'bend':[0.,1.2,1.17,0.,-.3,0.]}
events=[]
fake.desired_yaw=lambda:C.desired_yaw(fake)
fake.send_arm_action=lambda *a:events.append('bend')
fake.rotate_yaw=lambda a,cb:events.append(('rotate',a))
fake.alignment_failed=lambda reason:events.append(('failed',str(reason)))
C.align_then_bend(fake,lambda:None)
assert events==[('rotate',math.pi/2)], 'Arm bent before pedestal alignment'
events.clear(); fake.yaw_position=math.pi/2; C.align_then_bend(fake,lambda:None)
assert events==['bend']
events.clear(); fake.yaw_busy=True; C.align_then_bend(fake,lambda:None)
assert not events and callable(fake.yaw_waiter), 'Did not wait for prealignment'
fake.yaw_busy=False; fake.world_target['kind']='place'; C.align_then_bend(fake,lambda:None)
assert events[-1][0]=='failed', 'Mismatched task target allowed bending'
print('PASS: world direction, chassis heading compensation, reverse target, stale feedback, yaw-before-bend, pending yaw, mismatched target')
events.clear(); fake.world_target={'kind':'grab'}; fake.alignment_generation=3
fake.yaw_busy=True; fake.is_executing=True; fake.yaw_handle=SimpleNamespace(cancel_goal_async=lambda:events.append('cancel'))
fake._reset_execution=lambda:events.append('reset')
C.alignment_nav_status(fake,SimpleNamespace(data='paused'))
assert fake.world_target is None and fake.alignment_generation==4 and fake.yaw_waiter is None
assert events==['cancel','reset'], 'Pause left a pending arm sequence'
# A pause between send_goal and acceptance must also cancel the late goal.
events.clear(); accepted_callbacks=[]
fake.yaw_position=0.; fake.alignment_generation=4; fake.yaw_waiter=None
fake.get_logger=lambda:SimpleNamespace(info=lambda m:None,error=lambda m:None)
fake.yaw_client=SimpleNamespace(send_goal_async=lambda goal:SimpleNamespace(add_done_callback=lambda cb:accepted_callbacks.append(cb)))
C.rotate_yaw(fake,1.,lambda:events.append('bend'))
fake.alignment_generation+=1
handle=SimpleNamespace(accepted=True,cancel_goal_async=lambda:events.append('late_cancel'),get_result_async=lambda:SimpleNamespace(add_done_callback=lambda cb:None))
accepted_callbacks[0](SimpleNamespace(result=lambda:handle))
assert events==['late_cancel'], 'Late accepted goal survived pause'
print('PASS: pause cancels active and late-accepted pedestal goals and clears bending continuation')

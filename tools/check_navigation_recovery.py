import importlib.util
from types import SimpleNamespace as S
from unittest.mock import Mock, patch
spec=importlib.util.spec_from_file_location('navigator','/mnt/e/workspace/AIC/src/nav_simple/nav_simple/simple_navigator.py'); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
class Harness:
 recovery_callback=m.SimpleNav2Navigator.recovery_callback
 check_recovery=m.SimpleNav2Navigator.check_recovery
 def get_clock(self):return S(now=lambda:S(nanoseconds=100_000_000_000))
 def get_logger(self):return Mock()
 def __init__(self):
  self.recovering=False;self.recovery_started=None;self.recovery_ready_since=None;self.resume_target=None
  self.pending_operation=(3,8,'navigation_acceptance');self.current_goal_handle=Mock();self.goal_target=(1.,2.,3.);self.request_id=3;self.last_nav_feedback=9
  self.cancel_pending_plan=Mock();self.costmap_poses={'local':(100,10),'global':(100,10)};self.emergency_stop_active=False
  self.nav_client=Mock();self.nav_client.server_is_ready.return_value=True;self.plan_client=Mock();self.plan_client.server_is_ready.return_value=True
  self.status_pub=Mock();self.send_goal=Mock()
h=Harness(); old=h.current_goal_handle
with patch.object(m.time,'monotonic',return_value=10):h.recovery_callback(S(data=True))
assert h.request_id==4 and h.resume_target==(1.,2.,3.) and h.pending_operation is None
old.cancel_goal_async.assert_called_once()
with patch.object(m.time,'monotonic',return_value=10.1):assert h.check_recovery()
h.send_goal.assert_not_called()
h.recovery_callback(S(data=False))
with patch.object(m.time,'monotonic',return_value=10.2):h.check_recovery()
h.costmap_poses={'local':(100,12.3),'global':(100,12.3)}
with patch.object(m.time,'monotonic',return_value=12.3):h.check_recovery()
h.send_goal.assert_called_once_with(1.,2.,3.,5)
h=Harness()
with patch.object(m.time,'monotonic',return_value=10):h.recovery_callback(S(data=True));h.recovery_callback(S(data=False))
h.costmap_poses={'local':(98,10),'global':(100,10)}
with patch.object(m.time,'monotonic',return_value=10.1):h.check_recovery()
assert h.recovery_ready_since is None
h.resume_target=None;h.emergency_stop_active=True;h.costmap_poses={'local':(100,11),'global':(100,11)}
with patch.object(m.time,'monotonic',return_value=11):h.check_recovery()
h.costmap_poses={'local':(100,13.1),'global':(100,13.1)}
with patch.object(m.time,'monotonic',return_value=13.1):h.check_recovery()
h.send_goal.assert_not_called()
h=Harness()
with patch.object(m.time,'monotonic',return_value=10):h.recovery_callback(S(data=True))
with patch.object(m.time,'monotonic',return_value=131):h.check_recovery()
assert h.resume_target is None and h.status_pub.publish.call_args.args[0].data=='failed: navigation_recovery_timeout'
print('PASS: goal preservation, cancellation, readiness delay, stale pose fence, emergency stop, bounded timeout')

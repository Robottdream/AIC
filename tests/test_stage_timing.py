import json
from types import SimpleNamespace as NS
import unittest
from test_time_optimization import methods


class StageTimingTests(unittest.TestCase):
    def test_transition_uses_shared_monotonic_clock_and_sim_time(self):
        C=methods('main_controller/main_controller/main_controller_node.py',
                  'MainControllerNode', {'current_step'})
        node=C()
        node.current_step='WAIT_TASK'
        received=[]
        node.task_stage_pub=NS(publish=lambda msg:received.append(json.loads(msg.data)))
        node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=42))
        node.current_step='SELECT_BLOCK'
        node.current_step='SELECT_BLOCK'
        self.assertEqual(len(received),1)
        self.assertEqual(received[0]['previous'],'WAIT_TASK')
        self.assertEqual(received[0]['step'],'SELECT_BLOCK')
        self.assertEqual(received[0]['sim_ns'],42)
        self.assertGreater(received[0]['monotonic_ns'],0)
        self.assertEqual(node.current_step,'SELECT_BLOCK')


if __name__=='__main__':unittest.main()

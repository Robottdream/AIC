"""Known-track prediction regression checks without a ROS graph."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/nav_simple'))
from nav_simple.trajectory_prediction import predict_position, crossing_conflicts


class TrajectoryTests(unittest.TestCase):
    def test_diagonal_prediction_and_endpoint_reversal(self):
        track = dict(name='diagonal', points=(-2., -2., 2., 2.), size=.5, speed=.35)
        step = .35/math.sqrt(2)*.4
        samples = [(0., -step, -step), (.4, 0., 0.)]
        x, y = predict_position(track, samples, .4, 1.)
        self.assertAlmostEqual(x, .35/math.sqrt(2))
        self.assertAlmostEqual(x, y)
        for future in (10., 25., 100.):
            x, y = predict_position(track, samples, .4, future)
            self.assertAlmostEqual(x, y)
            self.assertLess(abs(x), 2.)

    def test_delay_resolves_real_crossing_conflict(self):
        track = dict(name='crossing', points=(-2., 0., 2., 0.), size=.5, speed=.35)
        histories = {'crossing': [(0., -.14, 0.), (.4, 0., 0.)]}
        path = [(0., -2.), (0., 2.)]
        self.assertEqual(crossing_conflicts(path, [track], histories, .4, 0., 1., 1.), {'crossing'})
        safe = [delay*.25 for delay in range(1, 33)
                if not crossing_conflicts(path, [track], histories, .4, delay*.25, 1., 1.)]
        self.assertTrue(safe)
        self.assertGreater(safe[0], 0.)
        self.assertFalse(crossing_conflicts([(4., -2.), (4., 2.)], [track], histories, .4, 0., 1., 1.))

    def test_stale_measurements_are_not_extrapolated(self):
        track = dict(name='crossing', points=(-2., 0., 2., 0.), size=.5, speed=.35)
        self.assertIsNone(predict_position(track, [(0., -.14, 0.), (.4, 0., 0.)], 1., 1.))


if __name__ == '__main__':
    unittest.main()

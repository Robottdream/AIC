"""Apply a measured speed trial through ROS parameter services (no file writes)."""
import argparse
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--speed', type=float, required=True)
    p.add_argument('--turn', type=float, required=True)
    p.add_argument('--accel', type=float, required=True)
    p.add_argument('--decel', type=float, required=True)
    args = p.parse_args()
    rclpy.init()
    node = rclpy.create_node('speed_trial_configuration')
    values = {
        'controller_server': {'FollowPath.desired_linear_vel': args.speed,
            'FollowPath.rotate_to_heading_angular_vel': args.turn,
            'FollowPath.max_angular_accel': 8.0, 'FollowPath.max_lookahead_dist': 1.0},
        'velocity_smoother': {'max_velocity': [args.speed, 0.0, 3.0],
            'max_accel': [args.accel, 0.0, 8.0], 'max_decel': [-args.decel, 0.0, -8.0]}}
    try:
        for target, parameters in values.items():
            client = node.create_client(SetParametersAtomically, '/'+target+'/set_parameters_atomically')
            assert client.wait_for_service(timeout_sec=10), target
            req = SetParametersAtomically.Request()
            req.parameters = [Parameter(name, value=value).to_parameter_msg() for name, value in parameters.items()]
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            assert future.result() and future.result().result.successful, (target, future.result())
            print(target, parameters, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

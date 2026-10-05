"""Apply a measured speed trial through ROS parameter services (no file writes)."""
import argparse
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParametersAtomically


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--speed', type=float, required=True)
    p.add_argument('--turn', type=float, default=2.0)
    p.add_argument('--accel', type=float, default=3.5)
    p.add_argument('--decel', type=float, default=5.0)
    p.add_argument('--speed-only', action='store_true', help='Preserve acceleration, turn and lookahead settings')
    args = p.parse_args()
    rclpy.init()
    node = rclpy.create_node('speed_trial_configuration')
    values = {
        'controller_server': {'FollowPath.desired_linear_vel': args.speed,
            'FollowPath.rotate_to_heading_angular_vel': args.turn,
            'FollowPath.max_angular_accel': 6.0, 'FollowPath.max_lookahead_dist': 1.0},
        'velocity_smoother': {'max_velocity': [args.speed, 0.0, 2.5],
            'max_accel': [args.accel, 0.0, 6.0], 'max_decel': [-args.decel, 0.0, -6.0]}}
    try:
        if args.speed_only:
            reader = node.create_client(GetParameters, '/velocity_smoother/get_parameters')
            assert reader.wait_for_service(timeout_sec=10), 'velocity_smoother'
            request = GetParameters.Request()
            request.names = ['max_velocity']
            pending = reader.call_async(request)
            rclpy.spin_until_future_complete(node, pending, timeout_sec=10)
            assert pending.done() and pending.result(), 'Cannot read current velocity limits'
            limits = list(pending.result().values[0].double_array_value)
            assert len(limits) == 3, limits
            limits[0] = args.speed
            values = {'controller_server': {'FollowPath.desired_linear_vel': args.speed},
                      'velocity_smoother': {'max_velocity': limits}}
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

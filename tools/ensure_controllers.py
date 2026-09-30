#!/usr/bin/env python3
"""Verify actual controller states, recovering from a lost spawn/load reply."""
import time

import rclpy
from controller_manager_msgs.srv import (
    ListControllers, LoadController, ConfigureController, SwitchController,
)

REQUIRED = ('joint_state_broadcaster', 'arm_controller', 'gripper_controller')


def main():
    rclpy.init()
    node = rclpy.create_node('ensure_aic_controllers')
    clients = {name: node.create_client(kind, '/controller_manager/' + name)
               for name, kind in [('list_controllers', ListControllers),
                                  ('load_controller', LoadController),
                                  ('configure_controller', ConfigureController),
                                  ('switch_controller', SwitchController)]}

    def call(name, request):
        client = clients[name]
        if not client.wait_for_service(timeout_sec=3.0):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=6.0)
        if not future.done():
            future.cancel()
            return None
        return future.result()

    def states():
        response = call('list_controllers', ListControllers.Request())
        return None if response is None else {c.name: c.state for c in response.controller}

    try:
        deadline = time.monotonic() + 80.0
        while time.monotonic() < deadline:
            current = states()
            if current is None:
                print('[控制器] 状态查询无响应，重试', flush=True)
                continue
            if all(current.get(name) == 'active' for name in REQUIRED):
                print('[控制器] 已验证三个控制器全部 active', flush=True)
                return 0
            for name in REQUIRED:
                # Read back state after each operation: a lost reply does not
                # imply that the operation failed, and must not cause a reload.
                current = states()
                if current is None:
                    break
                state = current.get(name)
                if state == 'active':
                    continue
                if state is None:
                    req = LoadController.Request(); req.name = name
                    call('load_controller', req)
                elif state == 'unconfigured':
                    req = ConfigureController.Request(); req.name = name
                    call('configure_controller', req)
                elif state == 'inactive':
                    req = SwitchController.Request()
                    req.activate_controllers = [name]
                    req.strictness = SwitchController.Request.STRICT
                    req.activate_asap = True
                    req.timeout.sec = 4
                    call('switch_controller', req)
                else:
                    print(f'[控制器] {name} 状态异常：{state}', flush=True)
                    return 1
                print(f'[控制器] {name} 原状态 {state}，操作后将复核', flush=True)
        print('[控制器] 未能验证全部 active，停止启动业务链', flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

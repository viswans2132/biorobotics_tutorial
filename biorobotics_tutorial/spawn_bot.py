#!/usr/bin/python3
import os
import sys
import rclpy
from gz_msgs.srv import SpawnEntity

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_node')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    content = sys.argv[1]
    init_x = float(sys.argv[2])
    init_y = float(sys.argv[3])
    init_z = float(sys.argv[4])

    req = SpawnEntity.Request()
    req.name = 'robot'
    req.xml = content
    req.robot_namespace = ''
    req.reference_frame = 'world'
    req.initial_pose.position.x = init_x
    req.initial_pose.position.y = init_y
    req.initial_pose.position.z = init_z

    while not cli.wait_for_service(timeout_sec = 1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result' + str(future.result().success) + ' ' + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
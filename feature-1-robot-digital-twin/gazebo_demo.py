import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import time


def main():
    # Start node
    rclpy.init()

    node = rclpy.create_node("demo_gazebo")

    #first spawn
    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")
    
    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Set data for request
    request = SpawnEntity.Request()
    request.name = 'cube'
    request.xml = open('object.sdf', 'r').read()
    request.robot_namespace = "demo"
    request.initial_pose.position.x = float(0.0)
    request.initial_pose.position.y = float(0.0)
    request.initial_pose.position.z = float(0.5)

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())
        

    #now move the object
    client2 = node.create_client(SetEntityState, "/world/set_entity_state")
    
    node.get_logger().info("Connecting to `/world/set_entity_state` service...")
    if not client2.service_is_ready():
        client2.wait_for_service()
        node.get_logger().info("...connected!")

    # Set data for request
    request2 = SetEntityState.Request()
    entitystate = EntityState()
    entitystate.name= "cube"
    request2.state = entitystate

    while True:
        request2.state.pose.position.x = 1.0
        request2.state.pose.position.y = 1.0
        request2.state.pose.position.z = 0.5
        
        node.get_logger().info("Sending service request to `/world/set_entity_state`")
        future2 = client2.call_async(request2)
        rclpy.spin_until_future_complete(node, future2)
        time.sleep(1)

        request2.state.pose.position.x = -1.0
        request2.state.pose.position.y = -1.0
        request2.state.pose.position.z = 0.5
        
        node.get_logger().info("Sending service request to `/world/set_entity_state`")
        future2 = client2.call_async(request2)
        rclpy.spin_until_future_complete(node, future2)
        time.sleep(1)

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


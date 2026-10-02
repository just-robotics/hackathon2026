"""Spawn configured fixtures without changing the simulator's static map."""
import json
from math import sin,cos
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import Bool,String
from .scenario_obstacles import load_obstacles,box_sdf


class SpawnObstacles(Node):
    def __init__(self):
        super().__init__('spawn_obstacles')
        self.declare_parameter('config_file','')
        self.declare_parameter('map_origin_x',-.468)
        self.declare_parameter('map_origin_y',-.582)
        self.cfg=load_obstacles(self.get_parameter('config_file').value)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready=self.create_publisher(Bool,'/simulation/obstacles_ready',qos)
        self.status=self.create_publisher(String,'/simulation/obstacles_status',qos)
        self.ready.publish(Bool(data=False))
        self.client=self.create_client(SpawnEntity,'/spawn_entity')
        self.boxes=self.cfg['boxes'] if self.cfg['enabled'] else []
        self.index=0;self.future=None;self.finished=False
        self.create_timer(.1,self.tick)

    def tick(self):
        if self.finished:return
        if self.future is not None:
            if not self.future.done():return
            response=self.future.result()
            if not response.success:
                self.get_logger().error(response.status_message)
                self.finished=True;return
            self.index+=1;self.future=None
        if self.index==len(self.boxes):
            self.ready.publish(Bool(data=True))
            self.status.publish(String(data=json.dumps(self.cfg)))
            self.get_logger().info(f'{self.index} unmapped boxes spawned; static /map unchanged')
            self.finished=True;return
        if not self.client.service_is_ready():return
        box=self.boxes[self.index];x,y,yaw=box['pose']
        request=SpawnEntity.Request();request.name=box['name'];request.reference_frame='world'
        request.xml=box_sdf(box)
        request.initial_pose.position.x=x+self.get_parameter('map_origin_x').value
        request.initial_pose.position.y=y+self.get_parameter('map_origin_y').value
        request.initial_pose.position.z=box['size'][2]/2
        request.initial_pose.orientation.z=sin(yaw/2);request.initial_pose.orientation.w=cos(yaw/2)
        self.future=self.client.call_async(request)


def main():
    rclpy.init();node=SpawnObstacles()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

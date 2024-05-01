import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    return LaunchDescription([joy_node])
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize the joint state subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/md80/joint_states',
            self.joint_state_callback,
            10
        )

        # Initialize the velocity command subscriber
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Current joint positions
        self.current_joint_positions = np.zeros(4)  # Example for 3 joints

        # Robot pose (initially at origin)
        self.robot_pose = [0.0, 0.0, 0.0]
    l_steer, l_drive, r_steer, r_drive = 0, 0, 0, 0
    r = 0.016
    #ls = 0.25
    lc = 0.40828 # derived from CAD P-G measurment 
    lo = 0.8
    #candle Id's
    # steerL = 200
    # driveL = 201
    # steerR = 203
    # driveR = 204
    vehicle_length = 0.8; # (m) 111in is 2.819m for 2016 Toyota Avalon
    vehicle_width  = 0.6; #(m) 62.4in = 1.585m for 2016 Toyota Avalon
    WheelRadi = 0.016;
    ls = 0.21; # ls half wheel axial
    lc = vehicle_length/2; #lc
    Alpha1 = math.atan2(ls,-lc);
    Alpha2 = math.atan2(-ls,-lc);
    Alpha3 = math.atan2(-ls,lc);
    Alpha4 = math.atan2(ls,lc);
    HomePosLeft = np.pi/2-Alpha1;
    HomePosRight = -3*np.pi/2-Alpha2;
    Lw = math.sqrt(ls**2+lc**2); #lw

    def Kinematics_ARC (r,ls,lc,lo,BV,lw,alpha1, alpha2, HomePosLeft, HomePosRight): #BV =body velocitys
        
        xdotd = BV[0]
        ydotd = BV[1]
        thtadotd = BV[2]

        vtangent = (lw * thtadotd)
        print(vtangent)
        vtangent1x = -vtangent * np.sin(alpha1)
        vtangent1y = vtangent * np.cos(alpha1)
        
        vtangent2x = -vtangent * np.sin(alpha2)
        vtangent2y = vtangent * np.cos(alpha2)
       
        vw1x = xdotd + vtangent1x
        vw1y = ydotd + vtangent1y
        print(vw1y)
        vw2x = xdotd + vtangent2x
        vw2y = ydotd + vtangent2y
       
        beta1cmd = np.pi + HomePosLeft + math.atan2(vw1y, vw1x)
        dPhi1cmd = -(1 / r) * math.sqrt((vw1x ** 2) + (vw1y ** 2)) 
        print(beta1cmd)
        print(dPhi1cmd)
        beta2cmd = np.pi + HomePosRight + math.atan2(vw2y, vw2x) 
        dPhi2cmd = -(1 / r) * math.sqrt(vw2x ** 2 + vw2y ** 2)
        print(beta2cmd)
        print(dPhi2cmd)
        return dPhi1cmd, dPhi2cmd, beta1cmd, beta2cmd


    def cmd_vel_callback(self, msg):
        # Placeholder for velocity command processing
        #pass
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        l_drive,r_drive,l_steer,r_steer  = Kinematics_ARC (r,ls,lc,lo,[vx,vy,omega],Lw,Alpha1, Alpha2, HomePosLeft, HomePosRight)
        self.get_logger().info('motor commands')
        self.get_logger().info(l_drive,r_drive,l_steer,r_steer)
    def joint_state_callback(self, msg):
        # Update current joint positions
        for i, name in enumerate(msg.name):
            if name == 'joint_name':  # Update according to your robot's joint name
                self.current_joint_positions[i] = msg.position[i]

        # Update robot pose
        # For this example, assume a simple 2D robot with x and y translations
        x, y = self.calculate_robot_position(self.current_joint_positions)
        self.robot_pose = [x, y, 0.0]  # Assume no rotation for simplicity

        # Publish TF transformation
        self.publish_tf_transform()

    def calculate_robot_position(self, joint_positions):
        # Placeholder for robot position calculation based on joint positions
        # Modify this function according to your robot's kinematics
        # For this example, assume a simple 2D robot with x and y translations
        x = joint_positions[0]  # Example: using first joint position as x coordinate
        y = joint_positions[1]  # Example: using second joint position as y coordinate
        return x, y

    def publish_tf_transform(self):
        # Publish the transform between the base link and the world frame
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'  # World frame
        transform.child_frame_id = 'base_link'  # Base link frame
        transform.transform.translation.x = self.robot_pose[0]
        transform.transform.translation.y = self.robot_pose[1]
        transform.transform.translation.z = 0.0  # Assume planar motion
        quat = tf2_geometry_msgs.Quaternion()
        quat.set_from_euler(0, 0, self.robot_pose[2])  # Convert yaw to quaternion
        transform.transform.rotation = quat
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    generate_launch_description()
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

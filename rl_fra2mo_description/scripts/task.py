#! /usr/bin/env python3
import rclpy
import time

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from visualization_msgs.msg import Marker
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from tf_transformations import quaternion_from_euler, euler_from_quaternion


def create_pose(x, y, z=0.0, yaw=0.0):
    """Crea un PoseStamped per una posizione specifica."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    # Orientamento in quaternioni (usando solo yaw)
    q = quaternion_from_euler(0.0, 0.0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


class TaskNode(Node):
    def __init__(self):
        super().__init__('task_node')

        # Broadcaster TF Statico
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Buffer e Listener per TF dinamiche
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher per il comando di velocità
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber per rilevare il marker (visualization_msgs/Marker)
        self.marker_pose = None
        self.marker_orientation = None
        self.create_subscription(Marker, '/aruco_detect/marker', self.marker_callback, 10)

    def marker_callback(self, msg: Marker):
        """Callback per il rilevamento del marker.
           Salva la posizione e l'orientamento del marker (nel frame camera)."""
        # Questi campi di solito sono riferiti al frame della camera
        self.marker_pose = msg.pose.position
        self.marker_orientation = msg.pose.orientation

    def publish_static_tf(self, position, orientation):
        """
        Pubblica una trasformazione statica per il marker **in map**.
        1. Creiamo un PoseStamped (in camera_link).
        2. Trasformiamo in map.
        3. Pubblichiamo un TF statico in map.
        """

        # 1. Creiamo un PoseStamped nel frame della camera
        marker_pose_camera = PoseStamped()
        marker_pose_camera.header.stamp = self.get_clock().now().to_msg()
        marker_pose_camera.header.frame_id = 'camera_link'  # Sostituisci col frame corretto se diverso
        marker_pose_camera.pose.position = position
        marker_pose_camera.pose.orientation = orientation

        # 2. Trasformiamo la posa da camera_link a map
        try:
            # Ottieni la trasformazione da map a camera_link
            transform_map_camera = self.tf_buffer.lookup_transform(
                'map',                              # target frame
                marker_pose_camera.header.frame_id, # source frame (camera_link)
                rclpy.time.Time()                   # tempo (ora)
            )
            # Applica la trasformazione
            marker_pose_map = do_transform_pose(marker_pose_camera, transform_map_camera)

        except Exception as e:
            self.get_logger().error(f"Impossibile trasformare la posa del marker in map. Errore: {e}")
            return

        # 3. Crea un TF statico rispetto a map
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'map'
        transform_stamped.child_frame_id = 'aruco_marker_frame'

        transform_stamped.transform.translation.x = marker_pose_map.pose.position.x
        transform_stamped.transform.translation.y = marker_pose_map.pose.position.y
        transform_stamped.transform.translation.z = marker_pose_map.pose.position.z
        transform_stamped.transform.rotation = marker_pose_map.pose.orientation

        self.static_tf_broadcaster.sendTransform(transform_stamped)
        self.get_logger().info("Published static TF for ArUco marker in map frame.")

    def stop_robot(self):
        """Invia un comando di stop al robot."""
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Robot stopped.")


def orient_towards_marker(navigator, obstacle_pose, marker_orientation):
    """Calcola l'orientamento verso il marker e orienta il robot mantenendo la posizione attuale."""
    # Calcola lo yaw del marker
    orientation_q = (
        marker_orientation.x,
        marker_orientation.y,
        marker_orientation.z,
        marker_orientation.w,
    )
    _, _, yaw = euler_from_quaternion(orientation_q)

    # Mantieni la posizione dell'obiettivo vicino all'ostacolo 9, cambiando solo l'orientamento
    pose_to_marker = create_pose(
        obstacle_pose.pose.position.x,
        obstacle_pose.pose.position.y,
        obstacle_pose.pose.position.z,
        yaw=yaw + 3.9,
    )
    navigator.goToPose(pose_to_marker)

    while not navigator.isTaskComplete():
        pass

    if navigator.getResult() == TaskResult.SUCCEEDED:
        print("Robot oriented towards marker.")
    else:
        print("Failed to orient towards marker.")


def main():
    rclpy.init()

    # Inizializza il nodo ROS2
    node = TaskNode()

    # Inizializza il navigatore
    navigator = BasicNavigator()

    # Waypoints specifici
    initial_pose = create_pose(0.0, 0.0)  # Posizione iniziale
    obstacle_9_pose = create_pose(4.4, -1.8, yaw=0.0)  # Vicino all'ostacolo 9

    # Activate Nav2
    navigator.waitUntilNav2Active()

    # Step 1: Vai alla posizione iniziale
    print("Going to initial position...")
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass
    if navigator.getResult() != TaskResult.SUCCEEDED:
        print("Failed to reach initial position!")
        exit(1)

    # Step 2: Vai in prossimità dell'ostacolo 9
    print("Moving to obstacle 9...")
    navigator.goToPose(obstacle_9_pose)
    while not navigator.isTaskComplete():
        pass
    if navigator.getResult() != TaskResult.SUCCEEDED:
        print("Failed to reach obstacle 9!")
        exit(1)

    # Step 3: Cerca il marker ArUco
    print("Looking for ArUco marker...")
    while rclpy.ok() and node.marker_pose is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Orienta il robot verso il marker
    orient_towards_marker(navigator, obstacle_9_pose, node.marker_orientation)

    # Pubblica il TF statico durante la pausa
    print("Publishing static TF (in map frame)...")
    node.publish_static_tf(node.marker_pose, node.marker_orientation)

    # Pausa di 5 secondi
    print("Pausing for 5 seconds...")
    time.sleep(5)

    # Step 4: Torna alla posizione iniziale
    print("Returning to initial position...")
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass
    if navigator.getResult() == TaskResult.SUCCEEDED:
        print("Task completed successfully!")
        # Ferma il movimento del robot
        node.stop_robot()
    else:
        print("Failed to return to initial position!")

    # Mantenere il nodo attivo
    print("Node will remain active. Press Ctrl+C to exit.")
    rclpy.spin(node)


if __name__ == '__main__':
    main()


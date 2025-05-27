#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

class RobotActuator:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.class_sub = rospy.Subscriber("/predicted_class", String, self.class_callback, queue_size=1)
        
        # Lògica del "Give_Way"
        self.last_class = None
        self.contador = 0

        # Odometria
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Posició del senyal
        self.signal_x = None  # TODO
        self.signal_y = None  # TODO
        self.last_detection_time = None
        self.action_distance_threshold = 0.4 # 40 cm
        self.detection_timeout = 10.0

        rospy.loginfo("Node robot_actuator activo. Esperando clases...")
    
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def class_callback(self, msg):
        class_name = msg.data
        rospy.loginfo(f"Clase recibida: {class_name}")
        self.last_detection_time = rospy.Time.now()


        # Calcular distància al senyal i actuar
        if self.signal_x is not None and self.signal_y is not None and self.last_detection_time is not None:
            time_diff = rospy.Time.now() - self.last_detection_time
            if time_diff.to_sec() < self.detection_timeout:
                distance_to_signal = np.sqrt((self.robot_x - self.signal_x)**2 + (self.robot_y - self.signal_y)**2)
                rospy.loginfo(f"Distancia al senyal {class_name}: {distance_to_signal:.2f} metres")

                if distance_to_signal <= self.detection_distance_threshold:
                    twist_msg = Twist()
                    # Lògica de moviment
                    if class_name == "Stop":
                        twist_msg.linear.x = 0.0
                        twist_msg.angular.z = 0.0
                        self.cmd_vel_pub.publish(twist_msg)
                    elif class_name == "Turn_Right":
                        twist_msg.linear.x = 0.1
                        twist_msg.angular.z = -0.3
                        self.cmd_vel_pub.publish(twist_msg)
                    elif class_name == "Turn_Left":
                        twist_msg.linear.x = 0.1
                        twist_msg.angular.z = 0.3
                        self.cmd_vel_pub.publish(twist_msg)
                    elif class_name == "Give_Way":
                        if self.contador <= 3:
                            twist_msg.linear.x = 0.0
                            twist_msg.angular.z = 0.0
                            self.contador += 1
                            self.cmd_vel_pub.publish(twist_msg)
                        else:
                            twist_msg.linear.x = 0.3
                            twist_msg.angular.z = 0.0
                            self.cmd_vel_pub.publish(twist_msg)

                    
                if class_name != "Give_Way":
                    self.contador = 0

                self.last_class = class_name

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("robot_actuator_node_2")
    try:
        actuator = RobotActuator()
        actuator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node robot_actuator parado.")
    except Exception as e:
        rospy.logerr(f"Excepción no controlada en robot_actuator_node: {e}")

# TODO: Afegir odometria del robot amb el seu callback (move2) cordenades de la señal y nms fetectar señal a 40 cm
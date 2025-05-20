#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RobotActuator:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.class_sub = rospy.Subscriber("/predicted_class", String, self.class_callback, queue_size=1)
        
        self.last_class = None # Per a la lògica del "Give_Way"

        self.contador = 0

        rospy.loginfo("Node robot_actuator activo. Esperando clases...")

    def class_callback(self, msg):
        class_name = msg.data
        rospy.loginfo(f"Clase recibida: {class_name}")

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
    rospy.init_node("robot_actuator_node")
    try:
        actuator = RobotActuator()
        actuator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node robot_actuator parado.")
    except Exception as e:
        rospy.logerr(f"Excepción no controlada en robot_actuator_node: {e}")

# TODO: Afegir odometria del robot amb el seu callback (move2) cordenades de la señal y nms fetectar señal a 40 cm
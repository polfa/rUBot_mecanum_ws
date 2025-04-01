#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

pub = None
d = 0
vx = 0
vy = 0
vf = 0
message = None
far_move = None

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2

max_range = int(360 * scanRangesLengthCorrectionFactor)
back_min = int(330 * scanRangesLengthCorrectionFactor)
back_max = int(30 * scanRangesLengthCorrectionFactor)
bright_min = int(30 * scanRangesLengthCorrectionFactor)
bright_max = int(90 * scanRangesLengthCorrectionFactor)
right_min = int(90 * scanRangesLengthCorrectionFactor)
right_max = int(120 * scanRangesLengthCorrectionFactor)
fright_min = int(120 * scanRangesLengthCorrectionFactor)
fright_max = int(170 * scanRangesLengthCorrectionFactor)
front_min= int(170 * scanRangesLengthCorrectionFactor)
front_max = int(190 * scanRangesLengthCorrectionFactor)
fleft_min = int(190 * scanRangesLengthCorrectionFactor)
fleft_max = int(240 * scanRangesLengthCorrectionFactor)
left_min = int(240 * scanRangesLengthCorrectionFactor)
left_max = int(300 * scanRangesLengthCorrectionFactor)
bleft_min = int(300 * scanRangesLengthCorrectionFactor)
bleft_max = int(330 * scanRangesLengthCorrectionFactor)
        

class Regions:
    def __init__(self, msg):
        self.bright = min(min(msg.ranges[bright_min:bright_max]), 3)
        self.right =  min(min(msg.ranges[right_min:right_max]), 3)
        self.fright = min(min(msg.ranges[fright_min:fright_max]), 3)
        self.front =  min(min(msg.ranges[front_min:front_max]), 3)
        self.back =  min(min(msg.ranges[back_min:max_range]), min(msg.ranges[0:back_max]), 3)
        self.fleft = min(min(msg.ranges[fleft_min:fleft_max]), 3)
        self.left = min(min(msg.ranges[left_min:left_max]), 3)
        self.bleft = min(min(msg.ranges[bleft_min:bleft_max]), 3)

    def get_min_distance(self, d):
        # Lista de las regiones con su nombre y distancia
        regions = {
            "bright": self.bright,
            "right": self.right,
            "fright": self.fright,
            "front": self.front,
            "back": self.back,
            "fleft": self.fleft,
            "left": self.left,
            "bleft": self.bleft
        }

        # Filtrar las distancias menores que 'd'
        valid_regions = {region: distance for region, distance in regions.items() if distance < d}

        if valid_regions:
            # Obtener la región con la distancia mínima entre las válidas
            min_region = min(valid_regions, key=valid_regions.get)
            return f"{min_region}"
        else:
            return False

class Movement:
    def __init__(self):
        self.linear_x = 0
        self.linear_y = 0
        self.state_description = ''
        self.last_movement = None
    
    def set_linear_x(self, vx):
        self.linear_x = vx
    
    def set_linear_y(self, vy):
        self.linear_y = vy
    
    def set_description(self, d):
        self.state_description = d

def clbk_laser(msg):
    global message
    message = msg


def take_action(regions):
    global far_move
    msg = Twist()
    movement = Movement()
    min_distance = regions.get_min_distance(d)

    if min_distance is False and far_move is None:
        movement.set_description('case 1 - nothing')
        movement.set_linear_x(vx)  
        movement.set_linear_y(0)  
    elif min_distance == "front":
        movement.set_description('case 2 - front')
        movement.set_linear_x(0)
        movement.set_linear_y(vy)
        far_move = (vx, 0) 
    elif min_distance == "fright":
        movement.set_description('case 3 - fright')
        movement.set_linear_x(vx)
        movement.set_linear_y(vy/2)
        far_move = (vx, -vy/2)
    elif min_distance == "right":
        movement.set_description('case 4 - right')
        movement.set_linear_x(vx)
        movement.set_linear_y(0)
        far_move = (0, -vy) 
    elif min_distance == "bright":
        movement.set_description('case 5 - bright')
        movement.set_linear_x(vx)
        movement.set_linear_y(-vy/2)
        far_move = (-vx, -vy/2) 
    elif min_distance == "back":
        movement.set_description('case 6 - back')
        movement.set_linear_x(0)
        movement.set_linear_y(-vy)
        far_move = (-vx, 0) 
    elif min_distance == "bleft":
        movement.set_description('case 7 - bleft')
        movement.set_linear_x(-vx)
        movement.set_linear_y(-vy/2)
        far_move = (-vx, vy/2) 
    elif min_distance == "left":
        movement.set_description('case 8 - left')
        movement.set_linear_x(-vx)
        movement.set_linear_y(0)
        far_move = (0, vy) 
    elif min_distance == "fleft":
        movement.set_description('case 9 - fleft')
        movement.set_linear_x(-vx)
        movement.set_linear_y(vy/2)
        far_move = (vx, vy/2) 
    else:
        movement.set_description('case 10 - Far')
        movement.set_linear_x(far_move[0])
        movement.set_linear_y(far_move[1])

    rospy.loginfo(movement.state_description)
    msg.linear.x = movement.linear_x
    msg.linear.y = movement.linear_y
    pub.publish(msg)
    rate.sleep()

def shutdown():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Stop rUBot")

def main():
    global pub
    global sub
    global rate
    global d
    global vx
    global vy
    global vf    
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(25)

    d= rospy.get_param("~distance_laser")
    vx= rospy.get_param("~forward_speed")
    vy= rospy.get_param("~lateral_speed")
    vf= rospy.get_param("~speed_factor")

    msg = Twist()
    local_scan = None
    while not rospy.is_shutdown():
        if local_scan != message:
            local_scan = message
            if not isScanRangesLengthCorrectionFactorCalculated:
                scanRangesLengthCorrectionFactor = len(local_scan.ranges) / 360
                isScanRangesLengthCorrectionFactorCalculated = True
            regions = Regions(local_scan)
            take_action(regions)

    
    
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown()



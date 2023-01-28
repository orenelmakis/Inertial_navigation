#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
import numpy as np
from scipy.spatial.transform import Rotation as Rot

class position():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.ox = 0
        self.oy = 0
        self.oz = 0
        self.ow = 0


class husky_control:
    def __init__(self):
        self.position_husky = position()
        self.ground_truth_sub = rospy.Subscriber('/ground_truth', Odometry, self.ground_truth_position_callback)
        self.vel_control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.actionLib_server = actionlib.SimpleActionServer('husky_control', MoveBaseAction, self.actionLib_callback, False)
        self.actionLib_server.start()
        

    def ground_truth_position_callback(self, data):
        self.position_husky.ox = data.pose.pose.orientation.x
        self.position_husky.oy = data.pose.pose.orientation.y
        self.position_husky.oz = data.pose.pose.orientation.z
        self.position_husky.ow = data.pose.pose.orientation.w
        self.position_husky.x = data.pose.pose.position.x
        self.position_husky.y = data.pose.pose.position.y
        self.position_husky.z = data.pose.pose.position.z

    def actionLib_callback(self, goal):
        vel_msg = Twist()
        while not self.arrived_at_goal(goal):
            dis = np.linalg.norm(np.array([self.position_husky.x, self.position_husky.y]) - np.array([goal.target_pose.pose.position.x, goal.target_pose.pose.position.y]))
            R = Rot.from_quat([self.position_husky.ox, self.position_husky.oy, self.position_husky.oz, self.position_husky.ow])
            yaw, _, _ = R.as_euler('zyx')
            err_yaw = np.arctan2(goal.target_pose.pose.position.y - self.position_husky.y, goal.target_pose.pose.position.x - self.position_husky.x) - yaw
            if err_yaw > np.pi:
                err_yaw -= 2*np.pi
            elif err_yaw < -np.pi:
                err_yaw += 2*np.pi

            vel_msg.linear.x = dis*np.cos(err_yaw)
            vel_msg.angular.z = -np.sin(err_yaw)*np.cos(err_yaw)-(err_yaw)
            self.vel_control_pub.publish(vel_msg)
        result = MoveBaseResult()
        self.actionLib_server.set_succeeded(result)



    
    def arrived_at_goal(self, goal):
        diff = np.linalg.norm(np.array([self.position_husky.x, self.position_husky.y]) - np.array([goal.target_pose.pose.position.x, goal.target_pose.pose.position.y]))
        if  diff < 0.5:
            return True
        else:
            return False

        

def main():
    rospy.init_node('husky_control', anonymous=True)
    husky = husky_control()
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass



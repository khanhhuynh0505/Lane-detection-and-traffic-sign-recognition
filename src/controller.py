#!/usr/bin/python3

 
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

turned = False
areaMax = 0
areaFlag = False
sign = ""
STOP = False
MAXSPEED = 0.08
TOPSPEED = 0.09

class ControlLane():
    def __init__(self):
        self.sub_lane = rospy.Subscriber('/detect/lane', Float64, self.cbFollowLane, queue_size = 1)
        # self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
        self.sub_area = rospy.Subscriber('/yolov8/area', String, self.areaCheck, queue_size = 1)
        self.sub_sign = rospy.Subscriber('/yolov8/sign', String, self.signCheck, queue_size = 1)

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        
        self.lastError = 0
        self.prevError = 0
        self.angular_z = 0
        self.MAX_VEL = 0.1
        self.prev_time = time.time()
        self.integral=0
        rospy.on_shutdown(self.stop)

    # def cbGetMaxVel(self, max_vel_msg):
    #     self.MAX_VEL = max_vel_msg.data
    def signCheck(self, sign_msg):
        global sign
        sign = sign_msg.data

    def areaCheck(self, area):
        global areaMax
        global areaFlag
        areaMax = int(area.data)
        # print(areaMax)
        if(areaMax > 1000):
            areaFlag = True
        else:
            areaFlag = False

    def cbFollowLane(self, desired_center):
        #test
        # if(turned == False):    
        #     self.turnLeft()

        center = desired_center.data
        
        Kp = 0.001
        Ki=0.0001
        Kd = 0.0025
        
        error = center - 160
        current_time = time.time()

        dt=current_time-self.prev_time
        self.integral= (self.integral + error)*dt

        self.angular_z += Kp * (error - self.lastError) + Kd * (error - 2 * self.prevError + self.lastError) +Ki *self.integral
        
        self.prevError = self.lastError
        self.lastError = error


        twist = Twist()
        
        # self.goAhead(2)
        # self.turnLeft()
        # global STOP
        # STOP = True
        # print(sign)
        # if(areaFlag == True):
        #     twist.linear.x = 0
        # else:
        #     twist.linear.x = MAXSPEED
        if(areaFlag == True):
            #stop 3s
            
            start = rospy.Time.now()
            while(rospy.Time.now() - start < rospy.Duration(4)):
                # print(sign)
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                self.pub_cmd_vel.publish(twist)
            rospy.loginfo("Sign detected: %s", sign)
            #ahead 5s
            # start = rospy.Time.now()
            # while(rospy.Time.now() - start < rospy.Duration(5)):
            #     twist.linear.x = MAXSPEED
            #     twist.linear.y = 0
            #     twist.linear.z = 0
            #     twist.angular.x = 0
            #     twist.angular.y = 0
            #     twist.angular.z = 0
            #     self.pub_cmd_vel.publish(twist)
            if(sign == "left"):
                self.turnLeft()
                # self.centerCheck(desired_center)
            elif(sign == "right"):
                self.turnRight()
            elif(sign == "ahead"):
                self.goAhead()
            else:
                # self.goAhead()
                self.stop()
                global  STOP
                STOP = True
        else:
            twist.linear.x = MAXSPEED
        if(STOP == True):
            twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -max(self.angular_z, -1.0) if self.angular_z < 0 else -min(self.angular_z, 1.0)
        print(twist.angular.z)
        self.pub_cmd_vel.publish(twist)
        self.prev_time=current_time
        #time.sleep(0.3)

    


    def stop(self):
        twist = Twist()
        start = rospy.Time.now()
        while(rospy.Time.now() - start < rospy.Duration(1)): 
            twist.linear.x = 0.08
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub_cmd_vel.publish(twist)
        global STOP
        STOP = True
        rospy.loginfo("Stopped")
    
    def turnLeft(self):
        rospy.loginfo("Turning left")
        twist = Twist()
        start = rospy.Time.now()
        #straight 10s then turn left
        while(rospy.Time.now() - start < rospy.Duration(13)):
            twist.linear.x = TOPSPEED
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub_cmd_vel.publish(twist)
        start = rospy.Time.now()
        while(rospy.Time.now() - start < rospy.Duration(5)): 
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0.3
            self.pub_cmd_vel.publish(twist)
        start = rospy.Time.now()
        while(rospy.Time.now() - start < rospy.Duration(6)): 
            twist.linear.x = MAXSPEED
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub_cmd_vel.publish(twist)
        
        start = rospy.Time.now()
        while(rospy.Time.now() - start < rospy.Duration(2)): 
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub_cmd_vel.publish(twist)

        start = rospy.Time.now()
        while(rospy.Time.now() - start < rospy.Duration(2)): 
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = -0.025
            self.pub_cmd_vel.publish(twist)
        rospy.loginfo("Finished")
        turned = True
        
    def turnRight(self):
        rospy.loginfo("Turning Right")
        twist = Twist()
        start = rospy.Time.now()
        #straight 10s then turn right
        while(rospy.Time.now() - start < rospy.Duration(10)):
            twist.linear.x = TOPSPEED
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub_cmd_vel.publish(twist)
        start = rospy.Time.now()
        while(rospy.Time.now() - start < rospy.Duration(6)): 
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = -0.3
            self.pub_cmd_vel.publish(twist)
        start = rospy.Time.now()
        while(rospy.Time.now() - start < rospy.Duration(3)): 
            twist.linear.x = MAXSPEED
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub_cmd_vel.publish(twist)
        rospy.loginfo("Finished")
        turned = True
    
    def goAhead(self):
        rospy.loginfo("Going ahead")
        twist = Twist()
        start = rospy.Time.now()
        #straight 10s then turn left
        while(rospy.Time.now() - start < rospy.Duration(15)):
            twist.linear.x = TOPSPEED
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub_cmd_vel.publish(twist)
        rospy.loginfo("Finished")
        turned = True

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_lane')
    rospy.loginfo('lane following controller is running')
    node = ControlLane()
    #node.turnLeft()
    node.main()
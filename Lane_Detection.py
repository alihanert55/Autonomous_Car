#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
import time
from math import cos, sin, radians, degrees, atan2 ,asin , sqrt , pi
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from carAttribute import car_attribute
from smart_can_msgs.msg import FB_VehicleSpeed
from sensor_msgs.msg import Imu



class Lane():


    def __init__(self):
        rospy.init_node('LaneFollower',anonymous=True)
        rospy.Subscriber("/beemobs/FB_VehicleSpeed",FB_VehicleSpeed,self.FB_callback)
        rospy.Subscriber("/zed2/zed_node/imu/data", Imu, self.imu_callback)
        rospy.Subscriber('/zed2/zed_node/rgb_raw/image_raw_color', Image,self.image_callback)
        
        #car_xy_oriantation_attributes
        self.carx = 0.0
        self.cary = 0.0
        self.carx_error = 0.0
        self.cary_error = 0.0
        self.orientation_q = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.heading = 0.0
        
        #Twists
        self.car_attribute =  car_attribute()

        #car_line_attributes
        self.cv_image = None
        self.car_line = None
        self.car_line_error = list([None] * 25)
        self.distance_of_the_line = 0
        self.distance_of_the_middle = 0
        self.middle_point = 550
        self.distance_list = []

        #target_point_attributes
        self.targetx = 0.0
        self.targety = 0.0
        self.target_list = list([])


        #for_control_time
        self.timerr= time.time()
        self.controller = True


        
    #Oriantation_functions

    def euler_from_quaternion(self,x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)

        return roll, pitch, yaw
    
    def current_heading(self,yaw):
        yaw_degrees = degrees(yaw)
        if yaw_degrees < 0:
            yaw_degrees += 360  # Pozitif açıya çevirme
        return yaw_degrees

    def imu_callback(self,data):
        self.orientation_q = data.orientation
        # Quaternion'u Euler açısına dönüştür
        (roll, pitch, yaw) = self.euler_from_quaternion(self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w)
        self.heading = self.current_heading(yaw)
        #rospy.loginfo(f"{self.currentDegree}")
    
    def FB_callback(self,data):
        self.speed_calc = data.FB_ReelVehicleSpeed_Ms
        #rospy.loginfo(f"{self.speed_calc}")
        self.odometry()
        

    def odometry(self):
        current_time = time.time()
        time.sleep(0.0000000000000000000000000000000000000000000000000000000000000000000001)
        derivative_of_time = time.time() - current_time
        r =  self.speed_calc*derivative_of_time
        theta = self.heading
        theta_rad = radians(theta)
        self.carx_error = self.carx_error + r * cos(theta_rad)
        self.carx = 600*self.carx_error
        self.cary_error = self.cary_error + r * sin(theta_rad)
        self.cary=600*self.cary_error
        rospy.loginfo(f"carx = {self.carx} cary = {self.cary}")
        
    
    #Image_functions
   
    def image_callback(self,data):
        current_time = time.time()
        bridge=CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        self.cv_image = cv2.resize(self.cv_image , (1280,720))
        self.process(self.cv_image)
        self.run()

    
    def calculate_vertical_distance_to_line(self,point, line):
        px, py = point
        x1, y1, x2, y2 = line
        # y değerleri aynı olan ve çizgi üzerindeki nokta (x, py) bulunur
        if y1 != y2:
            m = (x2 - x1) / (y2 - y1)  # Eğimi hesapla
            c = x1 - m * y1  # x-kesim noktasını hesapla
            x_on_line = m * py + c  # py için x değerini hesapla
        else:
            # Çizgi yatay ise y1 ve y2 aynı olur
            x_on_line = x1
        # py, px noktasından çizgi üzerindeki (x_on_line, py) noktasına olan yatay uzaklık
        distance = abs(px - x_on_line)
        return distance, (x_on_line, py)



    def distance_of_real(self,point_of_car,lengths_with_lines_right,lengths_with_lines_left):
        distance_of_the_line_right = self.find_closest_lines(point_of_car,lengths_with_lines_right)[0][1]
        distance_of_the_line_left = self.find_closest_lines(point_of_car,lengths_with_lines_left)[0][1]
        if not (np.isinf(distance_of_the_line_right)) and not (np.isinf(distance_of_the_line_left)):
            if 500 <((distance_of_the_line_right + distance_of_the_line_left)/2) <700:
                self.middle_point = (distance_of_the_line_right + distance_of_the_line_left)/2
                self.distance_list.append(((distance_of_the_line_right - distance_of_the_line_left)/2))
            #rospy.loginfo(f"{((distance_of_the_line_right - distance_of_the_line_left)/2)}")
        #rospy.loginfo(f"mid_point = {self.middle_point}")
        if not (np.isinf(distance_of_the_line_left)) :
            self.distance_list.append(self.middle_point -distance_of_the_line_left)
            #rospy.loginfo(f"left = {(self.middle_point -distance_of_the_line_left)}")  
        if not (np.isinf(distance_of_the_line_right))  :
            self.distance_list.append(distance_of_the_line_right - self.middle_point)
           # rospy.loginfo(f"right = {(distance_of_the_line_right - self.middle_point)}") 
        #self.distance_list.append(self.distance_of_the_middle * 0.8)
        #rospy.loginfo(f"near={(self.distance_of_the_middle * 0.8)}")
        self.distance_list = self.near_distance()
        self.distance_of_the_middle = self.distance_list[0]
        self.distance_list = []

    def near_distance(self):
         return sorted(self.distance_list, key=lambda x: abs(x - self.distance_of_the_line))


    def change_in_car_line(self,set1 ,set2 , car_line):
        try:
            if len(set1)> 1 and len(set2) <= 1:
                return  "right"
            elif len(set2)> 1 and len(set1) <= 1:
                return  "left"
            else:
                    return None
        except:
            return None


    def calculate_distance_point_to_line(self,point, line):
        px, py = point
        x1, y1, x2, y2 = line
        # Çizgi denklemi (Ax + By + C = 0) parametreleri
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2

        # Nokta-çizgi mesafesi formülü
        distance = abs(A * px + B * py + C) / sqrt(A ** 2 + B ** 2)
        return distance



    def find_closest_lines(self,points, lines):
        try:
            distance_of_p = []
            for point in points:
                closest_distance = float('inf')
                closest_line = None
                for line in lines:
                    line_coordinates = line[1][0]
                    if len(points) != 1:
                        distance = self.calculate_distance_point_to_line(point, line_coordinates)
                        if distance < closest_distance:
                            closest_distance = distance
                            closest_line = line_coordinates
                    else:
                        distance , _ = self.calculate_vertical_distance_to_line(point,line_coordinates)
                        if distance < closest_distance:
                            closest_distance = distance
                            closest_line = line_coordinates
                distance_of_p.append((point, closest_distance, closest_line))
            return distance_of_p
        except:
            return

    def region_of_interest(self,image,vertices):
        mask=np.zeros_like(image)
        match_mask_color = 255
        cv2.fillPoly(mask,vertices,match_mask_color)
        masked_image = cv2.bitwise_and(image,mask)
        return masked_image

    def set_add(self,distance_of_points):
        try:
            set_of_line = set()
            for p,c,line in distance_of_points:
                line_t = tuple(line)
                set_of_line.add(line_t)
            return set_of_line
        except:
            pass

    def calculate_length(self,line):
        x1, y1, x2, y2 = line[0]
        return sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def correctness_of_line(self,car_line_error,car_line):
        if not None in self.car_line_error:
            left_value = 0
            right_value=0
            for i in car_line_error:
                if i == "right" :
                    right_value += 1
                elif i == "left" :
                    left_value += 1
                else:pass
            if right_value == 0 and left_value == 0 :
                return car_line
            else:
                if left_value < right_value :
                    return "right"
                elif left_value > right_value :
                    return "left"
                else:
                    return car_line
        else:
            return None

    def draw_lines(self,lines,points,point_of_car):
        if lines is not None:
            #rospy.loginfo(f"{lines}")
            left_lines = []
            right_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                lx ,ly ,lx1, ly1 =0,495,390,415
                rx ,ry ,rx1, ry1 =830,390,1280,485
                left_slope = (ly1 - ly) / (lx1 -lx)
                right_slope = (ry1 - ry) / (rx1 - rx)
                slope = (y2 - y1) / (x2 - x1)
                if ( slope < 0 and slope < left_slope + 0.05):# Sol şerit
                    left_lines.append(line)
                elif (slope > 0 and slope > right_slope - 0.05 ):  # Sağ şerit
                    right_lines.append(line)
            lengths_with_lines_left = [(self.calculate_length(line), line) for line in left_lines]
            lengths_with_lines_right = [(self.calculate_length(line), line) for line in right_lines]
            lengths_with_lines_left.sort(key=lambda x: x[0], reverse=True)
            lengths_with_lines_right.sort(key=lambda x: x[0], reverse=True)
            lengths_with_lines_left = lengths_with_lines_left[:15]
            lengths_with_lines_right = lengths_with_lines_right[:15]
            distance_of_points_to_line_left = self.find_closest_lines(points,lengths_with_lines_left)
            set_of_left = self.set_add(distance_of_points_to_line_left)
            distance_of_points_to_line_right = self.find_closest_lines(points,lengths_with_lines_right)
            set_of_right = self.set_add(distance_of_points_to_line_right)
            temp_pose = self.change_in_car_line(set_of_left,set_of_right,self.car_line)
            if temp_pose != None : self.car_line_error.append(temp_pose)
            self.car_line_error = self.car_line_error[len(self.car_line_error)-150:len(self.car_line_error)]
            if self.car_line == None: self.car_line = self.correctness_of_line(self.car_line_error,self.car_line)
            #rospy.loginfo(f"lane = {self.car_line}")
            #rospy.loginfo(f"lane_list {self.car_line_error}")
            self.distance_of_real(point_of_car,lengths_with_lines_right,lengths_with_lines_left)
            self.distance_of_the_line = (self.distance_of_the_middle)*0.2955
            #rospy.loginfo(f"distance = {self.distance_of_the_line}")


    def process(self,image):
        points = [(640, 400), (640, 440), (640, 480), (640, 560), (640, 640), (640, 720)]
        point_of_car = [(640, 720)]
        region_of_interest_vertices = [(0,720),(0,625) ,(535,400) ,( 705,400 ) ,(1280,625),(1280,720)]
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        grad_x = cv2.Sobel(img_gray, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
        grad_y = cv2.Sobel(img_gray, cv2.CV_16S, 0, 1, ksize=3, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
        abs_grad_x = cv2.convertScaleAbs(grad_x)
        abs_grad_y = cv2.convertScaleAbs(grad_y)
        grad = cv2.addWeighted(abs_grad_x, 0.8, abs_grad_y, 0.8, 0)
        _, grad = cv2.threshold(grad, thresh=140, maxval=255, type=cv2.THRESH_BINARY)
        grad =cv2.GaussianBlur(grad, (5, 5), 2)
        cropped_image =self.region_of_interest(grad,np.array([region_of_interest_vertices],np.int32))
        lines = cv2.HoughLinesP(cropped_image, 1, np.pi / 180, threshold=140, minLineLength=50, maxLineGap=30)
        self.draw_lines(lines,points, point_of_car)


    #control_functions
    '''
    def calculate_new_position_from_orientation(self,x1, y1, r, orientation_degrees, current_line , distance_of_the_line ):
        try:
            if current_line == 'left':
                calculate_plus_degree = asin(-((300+distance_of_the_line)/100)/r)
                theta = orientation_degrees + degrees(calculate_plus_degree)

            else:
                theta = orientation_degrees + degrees(asin((-distance_of_the_line/100)/r))
                
        except:
            theta = orientation_degrees

        #rospy.loginfo(f"theata {theta}")
        theta_rad = radians(theta)
        
        x2 = x1 + r * cos(theta_rad)
        y2 = y1 + r * sin(theta_rad)

        #rospy.loginfo(f"x = {x2} y = {y2}")
        
        return x2,y2
    
    def target_bearing(self,k_target):


        target_vector = [self.targetx - self.carx, self.targety - self.cary]
        rot_angle = (-1) * (self.yaw + pi / 2)
        target_vector = [
            target_vector[0] * cos(rot_angle) - target_vector[1] * sin(rot_angle),
            target_vector[0] * sin(rot_angle) + target_vector[1] * cos(rot_angle)
            ]
        target_vector[0] = k_target * target_vector[0]
        target_vector[1] = k_target * target_vector[1]


        #rospy.loginfo(f"target = {target_vector[0]}")
        k_angle = -1
        #rospy.loginfo(f"target vector {target_vector[0]}")
        angle =   (k_angle * target_vector[0])
        #rospy.loginfo(f"car pose  ={self.carx,self.cary}")
        rospy.loginfo(f"angle  = s{angle}")
        if angle < 0:
            angle -= 5
        else:
            angle += 5
        self.car_attribute.Turn(angle)
        time.sleep(0.02)
        self.car_attribute.GAS(3)

    def check_target(self):   
        if abs(self.targetx - self.carx) <= 1 and abs(self.targety - self.cary) <= 1:
            self.target_list.pop(0)
            if len(self.target_list) != 0:
                new_x,new_y = self.target_list[0]
                self.targetx = new_x
                self.targety = new_y
                return True
            else:
                return False
        return True
    
    def run(self):
        
        if self.car_line == "left":
            if len(self.target_list) == 0:    
                self.target_list.append(self.calculate_new_position_from_orientation(self.carx,self.cary,6,self.heading,self.car_line,self.distance_of_the_line))
                self.target_list.append(self.calculate_new_position_from_orientation(self.carx,self.cary,10,self.heading,self.car_line,self.distance_of_the_line))
                self.targetx ,self.targety = self.target_list[0]
                rospy.loginfo(f"target_list = {self.target_list}")
                rospy.loginfo(f"current target = {self.targetx , self.targety}")
                while self.check_target():
                    #rospy.loginfo("burda")
                    self.target_bearing(10)
                self.car_line_error = list(["right"] * 150)
                self.car_line = "right"
        else:
            new_x, new_y  = self.calculate_new_position_from_orientation(self.carx,self.cary,3,self.heading,self.car_line,self.distance_of_the_line)
            self.targetx = new_x
            self.targety = new_y
            self.target_bearing(5)
    '''
    
    def calculate_steering_angle(self,distance, d_min=10, d_max=90, angle_min=5, angle_max=35):
        #interplasyon
        # Mesafenin normalize edilmesi
        normalized_distance = (abs(distance) - d_min) / (d_max - d_min)
        
        # Direksiyon açısını hesapla
        steering_angle = angle_min + normalized_distance * (angle_max - angle_min)
        
        if distance < 0 :
            return -steering_angle
        return steering_angle  
    
    
    def run(self):
        if abs(self.distance_of_the_line) <10 :
            angle_to_rotate=self.calculate_steering_angle(self.distance_of_the_line)
            rospy.loginfo(f"angle = {angle_to_rotate}")
            self.car_attribute.Turn(angle_to_rotate)
            time.sleep(0.02)
            self.car_attribute.GAS(3)
        else:
            self.car_attribute.SaveAngle()
            self.car_attribute.GAS(3)

    '''        
    def line_reset(self):
        self.car_line = None
        self.car_line_error = list([None] * 150)
        self.targetx = 0.0
        self.targety = 0.0
        self.target_list = list([])
    '''


if __name__ == "__main__":
    line = Lane()
    while True:
        rospy.spin()

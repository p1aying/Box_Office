#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import time
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class BoxbotSimpleCore:
    def __init__(self):
        rospy.init_node('boxbot_core', anonymous=True)
        
        # ROS 퍼블리셔/서브스크라이버
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        # tf 브로드캐스터
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        # 시리얼 통신 설정
        self.port = rospy.get_param('~port', '/dev/ttyUSB0') #USB0가 UART모듈이라는 설정이 필요없는가? 일단 잘됨 
        self.baud = rospy.get_param('~baud', 57600)
        self.ser = None
        
        # 오도메트리 데이터
        self.info = [0.0, 0.0, 0.0, 0.0, 0.0]  # x, y, theta, vx, vth
        self.packet = [0, 0, 0, 0, 0, 0]
        
        # 명령어 관련
        self.cmd_array = "L+000A+000\n"  # 초기 정지 명령
        self.last_linear_dir = 1
        self.last_angular_dir = 1
        
        # Rate 설정
        self.pub_rate = rospy.Rate(30)
        self.write_rate = rospy.Rate(20)
        
        # 시리얼 연결
        self.connect_serial()
        
        # 스레드 설정
        self.write_thread = None
        self.read_thread = None
    
    def connect_serial(self):
        """시리얼 연결"""
        while not rospy.is_shutdown():
            try:
                self.ser = serial.Serial(
                    self.port, 
                    self.baud, 
                    timeout=3
                )
                rospy.loginfo("시리얼 연결됨: %s" % self.port)
                break
            except serial.SerialException:
                rospy.logwarn("시리얼 연결 실패, 재시도 중...")
                time.sleep(1)
                continue
    
    def cmd_vel_callback(self, msg):
        """cmd_vel 콜백"""
        linear_vel = int(msg.linear.x * 1000)
        angular_vel = int(msg.angular.z * 1000)
        
        # 방향 저장
        if linear_vel > 0:
            self.last_linear_dir = 1
        elif linear_vel < 0:
            self.last_linear_dir = -1
            
        if angular_vel > 0:
            self.last_angular_dir = 1
        elif angular_vel < 0:
            self.last_angular_dir = -1
        
        # 프로토콜 형식
        if linear_vel != 0:
            linear_str = ("+" if linear_vel > 0 else "-") + "%03d" % abs(linear_vel)
        else:
            linear_str = "+000" if self.last_linear_dir >= 0 else "-000"
            
        if angular_vel != 0:
            angular_str = ("+" if angular_vel > 0 else "-") + "%03d" % abs(angular_vel)
        else:
            angular_str = "+000" if self.last_angular_dir >= 0 else "-000"
            
        self.cmd_array = "L%sA%s\n" % (linear_str, angular_str)
    
    def write(self):
        """write 스레드"""
        while not rospy.is_shutdown():
            try:
                if self.ser and self.ser.is_open:
                    self.ser.write(self.cmd_array.encode())
                self.write_rate.sleep()
            except Exception as e:
                rospy.logerr("쓰기 오류: %s" % e)
                self.handle_serial_error()
    
    def read(self):
        """read 스레드 - 단순화 버전"""
        while not rospy.is_shutdown():
            try:
                if self.ser and self.ser.is_open and self.ser.readable():
                    arr = self.ser.readline()
                    
                    if arr and arr[0] == 'a':
                        # 간단한 파싱 ->파싱이 복잡하니까 오도메트리에 에러가 발생
                        for i in range(6):
                            self.packet[i] = arr.find(chr(97+i))
                        
                        new_info = [0.0] * 5
                        for i in range(5):
                            if self.packet[i] != -1 and self.packet[i+1] != -1:
                                try:
                                    new_info[i] = float(arr[self.packet[i]+1:self.packet[i+1]])
                                except ValueError:
                                    continue
                        
                        # 간단한 각도 검사 - 2π 주기성 처리 (각도 검사 안하면 어떻게 됨?)
                        if abs(new_info[2] - self.info[2]) > 5.0:
                            # 각도 순환 감지 (약 π에서 -π로 또는 그 반대)
                            if new_info[2] > 0 and self.info[2] < 0:
                                if new_info[2] - self.info[2] > 5.0:  # info[2]는 세타값인데 로봇의 회전값을 의미함 -> 너무 커지면 큰 지진일어난거
                                    new_info[2] -= 2 * math.pi
                            elif new_info[2] < 0 and self.info[2] > 0:
                                if self.info[2] - new_info[2] > 5.0:  # 위와 동일함
                                    new_info[2] += 2 * math.pi
                        
                        self.info = new_info
                        rospy.loginfo("odometry: x=%.3f, y=%.3f, th=%.3f, vx=%.3f, vth=%.3f" % 
                                    (self.info[0], self.info[1], self.info[2], self.info[3], self.info[4]))
                            
            except Exception as e:
                rospy.logerr("읽기 오류: %s" % e)
                self.handle_serial_error()
    
    def handle_serial_error(self):
        """시리얼 오류 처리"""
        # 정지 명령 전송
        self.cmd_array = "L+000A+000\n"
        
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(self.cmd_array.encode())
                self.ser.close()
        except:
            pass
        
        # 재연결
        self.connect_serial()
    
    def run(self):
        """메인 실행 루프"""
        self.write_thread = threading.Thread(target=self.write)
        self.write_thread.daemon = True
        self.read_thread = threading.Thread(target=self.read)
        self.read_thread.daemon = True
        
        self.write_thread.start()
        self.read_thread.start()
        
        while not rospy.is_shutdown():
            try:
                current_time = rospy.Time.now()
                
                # tf.transformations 사용
                odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.info[2])
                
                # TF 브로드캐스트
                self.odom_broadcaster.sendTransform(
                    (self.info[0], self.info[1], 0.),
                    odom_quat,
                    current_time,
                    "base_footprint",
                    "odom"
                )
                
                # 오도메트리 발행하는 곳
                odom = Odometry()
                odom.header.stamp = current_time
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_footprint"
                odom.pose.pose = Pose(Point(self.info[0], self.info[1], 0.), 
                                     Quaternion(*odom_quat))
                odom.twist.twist = Twist(Vector3(self.info[3], 0, 0), 
                                       Vector3(0, 0, self.info[4]))
                
                self.odom_pub.publish(odom)
                self.pub_rate.sleep()
                
            except Exception as e:
                rospy.logerr("실행 오류: %s" % e)
                self.handle_serial_error()


if __name__ == '__main__':
    try:
        robot = BoxbotSimpleCore()
        robot.run()
    except rospy.ROSInterruptException:
        pass
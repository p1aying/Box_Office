#!/usr/bin/env python3
import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
from std_msgs.msg import Bool, Int16, String
from position import *  # position.py에서 Location 클래스 가져오기

SUCCESS = True
FAIL = False
ROTATION_SPEED = 0.26

class BoxService:
    def __init__(self):
        # 위치 정보 및 상태 초기화
        self.location = Location()
        self.station_position = None  # 원래 대기 위치
        self.robot_state = "대기 중"
        
        # Firebase 초기화
        self.cred = credentials.Certificate('/home/cal/catkin_ws/src/box_service/script/box_key.json')  #데이터 베이스 인증 키 경로 설정해놈
        firebase_admin.initialize_app(self.cred, {
            'databaseURL': 'https://test-3ecff-default-rtdb.firebaseio.com/'  # 우리 파이어베이스 realtimeDB URL
        })
        self.db_ref = db.reference()
        
        
        self.position_map = {               # position.py에 저장되어 있음
            "S9": self.location.position1,  # 재진
            "K9": self.location.position4,  # 명우
            "F1": self.location.position3,  # 지은
            "C4": self.location.position2   # 동우
        }
        
       
        self.name_map = {
            "S9": "재진",
            "K9": "명우",
            "F1": "지은",
            "C4": "동우"
        }
        
        # ROS 노드 초기화
        rospy.init_node('box_service')
        
        # 퍼블리셔 및 액션 클라이언트 설정
        self.pub_initPose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.status_publisher = rospy.Publisher('robot_status', String, queue_size=10)
        
        # Move Base 액션 클라이언트 설정 - 이제 action을 하는 주체가 우리 로봇이 됨(move_base 기반)
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)     #이거 못 찾으면 ㅈㄴ 뺑이쳐야함
        
        
        rospy.Timer(rospy.Duration(1.0), self.update_status) #로봇 제어 주기
        
        rospy.loginfo("Box-office가 초기화 되었습니다.")
    
    def update_status(self, event):
        #"""로봇 상태를 Firebase에 업데이트하는 함수"""
        self.status_publisher.publish(self.robot_state)
        self.db_ref.update({"cur_pos": self.robot_state})
    
    def rotate_one_lap(self, angular_z, time_seconds):
        #"""로봇을 제자리에서 회전시키는 함수""" -> 로봇 제자리에서 회전 시키는 이유는 로봇이 라이다로 global map 대비 local map 정보를 업데이트 시켜야하기 때문 
        rospy.loginfo("맵 업데이트를 위해 회전을 시작합니다...")
        t1 = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown(): 
            cmd = Twist()
            cmd.angular.z = angular_z
            self.pub_cmd.publish(cmd)
            t2 = rospy.Time.now().to_sec()
            
            if int(t2 - t1) >= time_seconds:
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                break
        
        rospy.loginfo("회전 완료")
        return SUCCESS
    
    def move_forward(self, speed, time_seconds):
        """로봇을 전진시키는 함수"""
        t1 = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = speed
            self.pub_cmd.publish(cmd)
            t2 = rospy.Time.now().to_sec()
            
            if int(t2 - t1) >= time_seconds:
                cmd.linear.x = 0.0
                self.pub_cmd.publish(cmd)
                break
        
        return SUCCESS
    
    def move_backward(self, time_seconds):
        """로봇을 후진시키는 함수"""
        rospy.loginfo(f"후진 시작 ({time_seconds}초)...")
        t1 = rospy.Time.now().to_sec()
        
        while not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = -0.09
            self.pub_cmd.publish(cmd)
            t2 = rospy.Time.now().to_sec()
            
            if int(t2 - t1) >= time_seconds:
                cmd.linear.x = 0.0
                self.pub_cmd.publish(cmd)
                break
        
        rospy.loginfo("후진 완료")
        return SUCCESS
    
    def init_position(self, pose):
        """로봇의 초기 위치를 설정하고 회전하는 함수"""
        cnt = 0
        while not rospy.is_shutdown():
            self.pub_initPose.publish(pose)
            rospy.sleep(1)
            cnt += 1
            if cnt == 2:
                # 한 바퀴 회전하여 주변 환경 스캔
                self.rotate_one_lap(-ROTATION_SPEED, 26)
                break
        return SUCCESS
    
    def action_goal(self, location):
        """목표 위치로 이동하는 함수"""
        try:
            self.move_base_client.wait_for_server()
            self.move_base_client.send_goal(goal=location)
            
            # 결과 기다리기
            self.move_base_client.wait_for_result()
            
            state = self.move_base_client.get_state()
            rospy.loginfo(f"[Result] State: {state}")
            
            if state == 3:  # SUCCEEDED
                return SUCCESS
            else:
                return FAIL
        except Exception as e:
            rospy.logerr(f"액션 목표 이동 중 오류 발생: {e}")
            return FAIL
    
    def force_stop(self):
        """로봇을 강제로 정지시키는 함수"""
        cmd = Twist()
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        
        self.pub_cmd.publish(cmd)
        return SUCCESS
    
    def navigate_to_position(self, position_code): #여기서 position에 좌표가 지정되어 있어야함
        """지정된 위치 코드로 이동하는 함수"""
        if position_code not in self.position_map:
            rospy.logerr(f"알 수 없는 위치 코드: {position_code}")
            return FAIL
        
        person_name = self.name_map.get(position_code, "알 수 없는 위치")
        destination = self.position_map[position_code]
        
        rospy.loginfo(f"{person_name}의 자리({position_code})로 이동을 시작합니다.")
        self.robot_state = f"{person_name}의 자리로 이동 중"
        
        # 목적지로 이동
        success = self.action_goal(destination)
        
        if success:
            rospy.loginfo(f"{person_name}의 자리({position_code})에 도착했습니다.")
            self.robot_state = f"{person_name}의 자리에 도착"
            return SUCCESS
        else:
            rospy.logerr(f"{person_name}의 자리({position_code})로 이동 실패")
            self.robot_state = "이동 실패"
            return FAIL
    
    def go_to_initial_position(self):
        """초기 위치로 돌아가는 함수"""
        rospy.loginfo("초기 위치로 돌아가기 위해 이동합니다.")
        self.robot_state = "초기 위치로 이동 중"
        
        # position5(초기 자리)로 이동
        success = self.action_goal(self.location.position5)
        
        if success:
            rospy.loginfo("초기 위치에 도착했습니다.")
            self.robot_state = "초기 위치에 도착"
            return SUCCESS
        else:
            rospy.logerr("초기 위치로 이동 실패")
            self.robot_state = "초기 위치 이동 실패"
            return FAIL
    
    def wait_for_call(self):
        """Firebase에서 호출 정보를 기다리는 함수"""
        rospy.loginfo("호출 정보를 기다리고 있습니다...")
        self.robot_state = "대기 중"
        
        while not rospy.is_shutdown():
            # who_call_robot 값을 확인
            who_call = self.db_ref.child("who_call_robot").get()
            
            # 호출이 들어왔다면 이동 시작
            if who_call and who_call in self.position_map:
                person_name = self.name_map.get(who_call, "알 수 없는 위치")
                rospy.loginfo(f"새로운 호출 감지: {who_call} ({person_name}의 자리)")
                
                # call_robot 플래그 설정
                self.db_ref.update({"call_robot": True})
                
                return who_call
            
            rospy.sleep(1.0)  #1초마다 확인
    
    def wait_for_loading(self):
        """물건 적재를 기다리는 함수"""
        rospy.loginfo("물건 적재를 기다리고 있습니다...")
        self.robot_state = "물건 적재 대기 중"
        
        while not rospy.is_shutdown():
            # moving 값을 확인
            moving = self.db_ref.child("moving").get()
            
            # 물건 적재가 완료되면 다음 단계로 진행
            if moving is True:
                rospy.loginfo("물건 적재 완료를 감지했습니다. 다음 위치로 이동하기 전에 후진 후 회전합니다.")
                
                # 물건 적재 후 먼저 후진
                self.move_backward(3)  # 3초간 후진
                
                # 후진 후 한바퀴 회전하여 맵 업데이트
                self.rotate_one_lap(ROTATION_SPEED, 13)  #180도 회전
                
                return SUCCESS
            
            rospy.sleep(1.0)  # 1초마다 확인
    
    def wait_for_return_signal(self):
        """귀환 신호를 기다리는 함수"""
        rospy.loginfo("원래 위치로 돌아가는 신호를 기다리고 있습니다...")
        
        while not rospy.is_shutdown():
            # go_to_station 값을 확인
            go_to_station = self.db_ref.child("go_to_station").get() #go_to_station의 값을 확인
            
            # 신호 도착했다면?
            if go_to_station is True:
                rospy.loginfo("원래 위치로 돌아가는 신호를 감지했습니다. 이동하기 전에 후진 후 회전합니다.")
                
                
                self.move_backward(2)  #신호 받고 로봇이 안전마진에서 움직이려고 하지않음 그래서 후진 후, 확실하게 움직이도록 추가함
                
                #글로벌 맵 대비 로컬맵이 휘어있어서 주변 확인용으로 회전 시킴 -> 회전 속도를 줄이면 좀 더 성능이 좋아지려나?
                self.rotate_one_lap(ROTATION_SPEED, 13) #180도 회전
                
                return SUCCESS
            
            rospy.sleep(1.0)  # 1초마다 확인
    
    def run_service(self):
    
        self.init_position(self.location.InitialPosition) #초기 위치 설정하고 시작
        self.station_position = self.location.InitialPosition
        
        while not rospy.is_shutdown():
            try:
                #1 초기화 -  who_call_robot을 초기화 하는 게 어플과 상관없어야됨
                self.db_ref.update({
                    "who_call_robot": "",  # 이 값을 먼저 초기화
                    "receiver": "",
                    "receiver_pos": "",
                    "sender": "",
                    "sender_pos": ""
                })
                
                who_call = self.wait_for_call() # 2. 호출 신호 대기
                
                #3 호출 위치로 이동
                sender_success = self.navigate_to_position(who_call)
                if not sender_success:
                    self.force_stop()
                    rospy.logwarn("발신자 위치로 이동 실패, 다시 대기 상태로 전환")
                    self.db_ref.update({
                        "call_robot": False,
                        "moving": False
                    })
                    continue
                
                # 4. 물건 적재 대기
                self.wait_for_loading()
                
                # 5. Firebase에서 수신자 위치 정보 가져오기
                receiver_pos = self.db_ref.child("receiver_pos").get()
                
                if not receiver_pos or receiver_pos not in self.position_map:
                    rospy.logerr(f"유효하지 않은 수신자 위치 코드: {receiver_pos}")
                    self.db_ref.update({
                        "moving": False,
                        "call_robot": False
                    })
                    continue

                # 6. 수신자 위치로 이동
                receiver_name = self.name_map.get(receiver_pos, "알 수 없는 위치")
                rospy.loginfo(f"수신자 {receiver_name}의 자리({receiver_pos})로 이동합니다.")
                
                receiver_success = self.navigate_to_position(receiver_pos)
                if not receiver_success:
                    self.force_stop()
                    rospy.logwarn("수신자 위치로 이동 실패, 다시 대기 상태로 전환")
                    self.db_ref.update({
                        "moving": False,
                        "call_robot": False
                    })
                    continue
                
                # 7. 이동 완료 후 상태 업데이트
                self.db_ref.update({"moving": False})
                
                # 8. 원래 위치로 돌아가기 전 신호 대기 -> 원래 위치로 가라는 go_to_station이 true가 될 때까지 대기
                self.wait_for_return_signal()
                
                # 9. 원래 위치로 이동 -> 파이어 베이스, VScode에서 디버깅 용으로 출력시키는 거임
                rospy.loginfo("원래 위치로 돌아갑니다.")
                self.robot_state = "대기 위치로 복귀 중"
                self.db_ref.update({"moving": True})
                
                # 초기 위치도 맵핑으로 move_base 형식으로 만들어 둠(initial_position이랑은 조금 틀림)
                initial_position_success = self.go_to_initial_position()
                if not initial_position_success:
                    self.force_stop()
                    rospy.logwarn("초기 위치로 이동 실패")
                
                # 10. 상태 초기화
                self.db_ref.update({
                    "call_robot": False,
                    "go_to_station": False,
                    "who_call_robot": "",  # 이 부분이 중요 -> 이게 초기화 안되서 로봇이 다시 로봇을 부른 곳으로 이동해버림
                    "receiver": "",
                    "receiver_pos": "",
                    "sender": "",
                    "sender_pos": ""
                })
                
                self.robot_state = "대기 중"
                rospy.loginfo("사이클 완료, 다음 호출 대기 중")
                
            except Exception as e:
                rospy.logerr(f"서비스 실행 중 오류 발생: {e}")
                self.force_stop()
                self.robot_state = "오류 발생"
                self.db_ref.update({
                    "moving": False,
                    "call_robot": False,
                    "go_to_station": False,
                    "who_call_robot": ""  # 오류 발생 시에도 초기화
                })
                rospy.sleep(5.0)


if __name__ == '__main__':
    try:
        box_service = BoxService()
        box_service.run_service()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"프로그램 실행 중 오류 발생: {str(e)}")
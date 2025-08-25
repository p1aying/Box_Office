#!/usr/bin/env python3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

class Location:
    def __init__(self):
        # 연구실 맵의 5개 목표지점(재진,동우,지은,명우)+초기 위치 지점, 모든 좌표값은 rviz 기반으로 작성했음 
        
        # 위치 1: 재진이 자리
        self.position1 = MoveBaseGoal()
        self.position1.target_pose.header.frame_id = 'map'
        self.position1.target_pose.pose.position.x = 4.24081993103
        self.position1.target_pose.pose.position.y = -0.22011205554
        self.position1.target_pose.pose.position.z = 0.0
        self.position1.target_pose.pose.orientation.x = 0.0
        self.position1.target_pose.pose.orientation.y = 0.0
        self.position1.target_pose.pose.orientation.z = -0.318271756277
        self.position1.target_pose.pose.orientation.w = 0.947999519597
        
        # 위치 2 동우 자리
        self.position2 = MoveBaseGoal()
        self.position2.target_pose.header.frame_id = 'map'
        self.position2.target_pose.pose.position.x = 4.35821247101
        self.position2.target_pose.pose.position.y = 0.865772485733
        self.position2.target_pose.pose.position.z = 0.0
        self.position2.target_pose.pose.orientation.x = 0.0
        self.position2.target_pose.pose.orientation.y = 0.0
        self.position2.target_pose.pose.orientation.z = 0.300200509069
        self.position2.target_pose.pose.orientation.w = 0.953876121074
        
        # 위치 3 지은이 자리
        self.position3 = MoveBaseGoal()
        self.position3.target_pose.header.frame_id = 'map'
        self.position3.target_pose.pose.position.x = 3.00819396973
        self.position3.target_pose.pose.position.y = 0.81251387596
        self.position3.target_pose.pose.position.z = 0.0
        self.position3.target_pose.pose.orientation.x = 0.0
        self.position3.target_pose.pose.orientation.y = 0.0
        self.position3.target_pose.pose.orientation.z = 0.450338133891
        self.position3.target_pose.pose.orientation.w = 0.89285808792
        
        # 위치 4 명우 자리
        self.position4 = MoveBaseGoal()
        self.position4.target_pose.header.frame_id = 'map'
        self.position4.target_pose.pose.position.x = 2.80275630951
        self.position4.target_pose.pose.position.y = 0.190763145685
        self.position4.target_pose.pose.position.z = 0.0
        self.position4.target_pose.pose.orientation.x = 0.0
        self.position4.target_pose.pose.orientation.y = 0.0
        self.position4.target_pose.pose.orientation.z = -0.841300967677
        self.position4.target_pose.pose.orientation.w = 0.540567000274
        
        # 위치 5 초기 자리
        self.position5 = MoveBaseGoal()
        self.position5.target_pose.header.frame_id = 'map'
        self.position5.target_pose.pose.position.x = 0.199999913573
        self.position5.target_pose.pose.position.y = -0.0100001506507
        self.position5.target_pose.pose.position.z = 0.0
        self.position5.target_pose.pose.orientation.x = 0.0
        self.position5.target_pose.pose.orientation.y = 0.0
        self.position5.target_pose.pose.orientation.z = -0.0124970941747
        self.position5.target_pose.pose.orientation.w = 0.999921908269

        # 초기 위치 설정 (로봇의 시작 위치와 방향)
        self.InitialPosition = PoseWithCovarianceStamped()
        self.InitialPosition.header.frame_id = 'map'
        self.InitialPosition.pose.covariance = (0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787)
        self.InitialPosition.pose.pose.position.x = 0.199999913573
        self.InitialPosition.pose.pose.position.y = -0.0100001506507
        self.InitialPosition.pose.pose.position.z = 0.0
        self.InitialPosition.pose.pose.orientation.x = 0.0
        self.InitialPosition.pose.pose.orientation.y = 0.0
        self.InitialPosition.pose.pose.orientation.z = -0.0124970941747
        self.InitialPosition.pose.pose.orientation.w = 0.999921908269
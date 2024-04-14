    #!/usr/bin/env python
import rospy
from symplanning import blockPlanning
from your_service_package.srv import YourService1, YourService2, YourService3
import numpy as np

class main_node():
    def __init__(self):
        self.init_state = np.zeros((6,6))
        self.goal_state = np.zeros((6,6))
        self.bp = blockPlanning(self.init_state, self.goal_state,1)
        self.plans = []
        self.actions = []

    def call_state_services(self):
        rospy.wait_for_service('check_state')
        
        try:
            check_state = rospy.ServiceProxy('check_state', YourService1)

            # Example calls, adjust arguments as necessary
            resp1 = check_state()

            rospy.loginfo("State responses:")
            rospy.loginfo("Check State Response: %s", resp1)

            return resp1

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def call_b2p_services(self):
        rospy.wait_for_service('block_2_pose')
        
        try:
            block_2_pose = rospy.ServiceProxy('block_2_pose', YourService2)

            # Example calls, adjust arguments as necessary
            resp2 = block_2_pose()

            rospy.loginfo("Service responses:")
            rospy.loginfo("Block 2 Pose Response: %s", resp2)
            return resp2

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def call_planner_services(self, init: np.array, goal: np.array):
        self.bp.newStartGoal(init, goal)
        self.plans, self.actions = self.bp.AstarSearch()

    def call_exec_services(self, pose, action_code):
        rospy.wait_for_service('exec')
        
        try:
            executioner = rospy.ServiceProxy('exec', YourService3)

            # Example calls, adjust arguments as necessary
            resp3 = executioner(pose, action_code)

            rospy.loginfo("Service responses:")
            rospy.loginfo("executioner Response: %s", resp3)
            return resp3

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def pick(self, color, end):
        ###Go to view point
        color_dict = {0: [x,y,z,w], 1:[x,y,z,w]}
        view_pose = color_dict[color]
        resp = self.call_exec_services(view_pose, 0)
        if resp:
            blocks = self.call_b2p_services()
            if blocks:
                resp2 = self.call_exec_services(blocks[0].pose, 1)
                if resp2:
                    resp3 = self.call_exec_services(end, 2)
                    if resp3:
                        return True
        return False
    
    def take(self, color, start):
        ###Go to view point
        color_dict = {0: [x,y,z,w], 1:[x,y,z,w]}
        place_pose = color_dict[color]
        resp = self.call_exec_services(start, 1)
        if resp:
            resp2 = self.call_exec_services(place_pose, 2)
            if resp2:
                return True
        return False
    
    def transfer(self, start, end):
        resp = self.call_exec_services(start, 1)
        if resp:
            resp2 = self.call_exec_services(end, 2)
            if resp2:
                return True
        return False

    def start(self):
        rospy.init_node('service_client_node', anonymous=True)
        self.goal_state = np.array([[1, 2, 2, 2, 1, 2], [1, 2, 1, 2, 1, 2], [2, 1, 1, 2, 1, 1], [1, 2 ,2, 1, 1, 2], [1, 2 ,2, 1, 1, 2], [1, 2, 1 ,1, 2, 1]],dtype=int)
        self.call_planner_services(self.init_state, self.goal_state)
        while self.plans:
            expt_state = self.plans.pop(0)
            action = self.actions.pop(0)
            color, start, end = action
            if start == (-1, -1):
                succ = self.pick(color, end)
            if end == (-1, -1):
                succ = self.take(color, start)
            else:
                succ = self.transfer(start, end)
            curr_state = self.call_state_services()
            if curr_state != expt_state:
                plans = self.call_planner_services(curr_state, self.goal_state)

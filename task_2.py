import rospy
import numpy as np
from copy import deepcopy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

"""
TO DO:

    - setup mavros_msgs/BatteryStatus.msg to monitor status of the battery and trigger ABORT
        -- need to handle with AUTO.LANDING if possible and alter state transition

    - mavros_msgs/ExtendedState.msg

"""

# global constants
VICON_OFFSET_M = 0.1
RATE_MS = 20
LAUNCH_HEIGHT = 1
LAND_HEIGHT = 1
EUCLID_THRESH = 0.075

# STATE0_<name> = '<name>'
STATE0_IDLE = "IDLE"
STATE0_ACTIVE = "ACTIVE"

# STATE1_<name> = '<name>'
STATE1_IDLE = "IDLE"
STATE1_READY = "READY"
STATE1_BUSY = "BUSY"
STATE1_ABORT = "ABORT"

# Task 2: test case [x, y, z, w]
TEST_T2 = [[0, 0, 0, 0]]

# helper functions
def isItNear(mavrosPose: PoseStamped, currPose: PoseStamped):
    
    result = False
    
    mavROSPose = np.array([[mavrosPose.pose.position.x],
                           [mavrosPose.pose.position.y],
                           [mavrosPose.pose.Position.z]])
    targetPose = np.array([[currPose.pose.position.x],
                           [currPose.pose.position.y],
                           [currPose.pose.Position.z]])
    
    if np.linalg.norm(mavrosPose - targetPose) < EUCLID_THRESH:
        result = True
    
    return result

class roscoreDrone_T2:
    
    def __init__(self, nodeName):
        
        # assign node name
        self.nodeName = nodeName
        
        # placeholders for service topics - similar to regular rostopics
        self.servLaunch = None
        self.servTest = None
        self.servLand = None
        self.servAbort = None
        
        # subscriber nodes
        self.subState = None 
        self.subPose = None
        
        # publisher node
        self.pubPoseLocal = None
        
        # internal FSM variables
        self.currState_0 = STATE0_IDLE
        self.currState_1 = STATE1_IDLE
        
        # initialize member variables
        self.rate = rospy.Rate(RATE_MS)
        self.mavrosState = State()
        self.mavrosPose = PoseStamped()
        self.goalPose = PoseStamped()
        
        # requests from callback functions
        self.reqLaunch = False
        self.reqTest = False
        self.reqLand = False
        self.reqAbort = False
        
        # Task 2: variables
        self.cntT2 = 0
        
        # waypoint navigation queue and variables
        self.newWayPtReq = False
        self.wayP2Nav = []
               
        return
         

    def _setup_mavROS(self):
        
        # initialize node
        rospy.init_node(self.nodeName)
        
        # initialize service nodes
        self.servLaunch = rospy.Service(self.nodeName + '/comm/launch', Empty, self.callback_launch)
        self.servTest = rospy.Service(self.nodeName + '/comm/test', Empty, self.callback_test)
        self.servLand = rospy.Service(self.nodeName + '/comm/land', Empty, self.callback_land)
        self.servAbort = rospy.Service(self.nodeName + '/comm/abort', Empty, self.callback_abort)
        
        # initialize subscriber nodes
        self.subState = rospy.Subscriber("mavros/state", State, callback = self._stateCallBack)
        self.subPose = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback = self._poseCallback)
            
        # initilaize publisher node
        self.pubPoseLocal = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # wait for flight controller connection
        while(not rospy.is_shutdown() and not self.currState.connected):
            self.rate.sleep()
            
        print("Connection Secured")
        
        # initialize starting position
        tempPose = deepcopy(self.mavrosPose)
        self.goalPose = tempPose    # maybe it works, maybe it doesn't
        
        return
    
    # callback for rosTopics
    def _stateCallBack(self, msg):
        self.mavrosState = msg
        return
    
    def _poseCallback(self, msg: PoseStamped):
        self.mavrosPose = msg
        return
    
    # Callback handlers
    def handle_launch(self):
        
        print('Launch Requested. Your drone should take off.')
        
        # check if we are ready for request
        if (self.currState_0 == STATE0_ACTIVE) and (self.currState_1 == STATE1_READY):
            self.reqLaunch = True
        
        return

    def handle_test(self):
        
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        
        # check if we are ready for request
        if (self.currState_0 == STATE0_ACTIVE) and (self.currState_1 == STATE1_READY):
            self.reqTest = True

        return
    
    def handle_land(self):
        
        print('Land Requested. Your drone should land.')
        
        # check if we are ready for request
        if (self.currState_0 == STATE0_ACTIVE) and (self.currState_1 == STATE1_READY):
            self.reqLand = True
        
        return

    def handle_abort(self):
        
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        
        # check if we are ready for request - request abort anytime since it is an emergency
        if (self.currState_0 == STATE0_ACTIVE) and (self.currState_1 != STATE1_ABORT):
            self.reqAbort = True

        return
    
    # Service callbacks
    def callback_launch(self, request):
        self.handle_launch()
        return EmptyResponse()

    def callback_test(self, request):
        self.handle_test()
        return EmptyResponse()

    def callback_land(self, request):
        self.handle_land()
        return EmptyResponse()

    def callback_abort(self, request):
        self.handle_abort()
        return EmptyResponse()
    
    ## CLASS METHODS
    
    def _runFSM(self):
            
        # change self.currState_1 to READY
        if self.currState_1 == STATE1_IDLE:
            self.currState_1 = STATE1_READY
            return
        
        # check Abort first for emergency landing and 
        if self.reqAbort:
            
            self.currState_1 = STATE1_ABORT
            
            # force the vehicle to land           
            self.currPose.pose.position.z = LAND_HEIGHT
            
            self.reqAbort = False
            
            return
                
        # check land request and add waypoint to the stack
        if self.reqLand:
            
            self.currState_1 = STATE1_BUSY
                      
            newPose = np.array([[self.currPose.pose.position.x],
                                [self.currPose.pose.position.y],
                                [LAND_HEIGHT]])
            
            self.wayP2Nav.append(newPose)
            
            self.reqLand = False
            
            return
        
        # check test request and popopulate the queue for navigation
        if self.reqTest:
            
            self.currState_1 = STATE1_BUSY
            
            # set way point to the test case or cases
            for i in range(0, len(TEST_T2), 1):
                
                newPose = np.array([[TEST_T2[i][0]],
                                    [TEST_T2[i][1]],
                                    [TEST_T2[i][2]]])
                                            
                self.wayP2Nav.append(newPose)
            
            self.reqTest = False
            
            return
        
        # check land request and add waypoint accordingly
        if self.reqLaunch:
            
            self.currState_1 = STATE1_BUSY
            
            # set way point to the launch position
            newPose = np.array([[self.currPose.pose.position.x],
                                [self.currPose.pose.position.y],
                                [LAUNCH_HEIGHT]])
            
            self.wayP2Nav.append(newPose)
            
            self.reqLaunch = False
            
            return
        
        return
    
    def _updatePose(self):
        
        # iterate through the waypoints if we are not asked to abort
        if self.currState_1 != STATE1_ABORT:
            
            if len(self.wayP2Nav) != 0:                
                
                if not self.newWayPtReq:
                    
                    nextWayPt = self.wayP2Nav[0]
                    
                    self.goalPose.position.x = nextWayPt[0][0]
                    self.goalPose.position.y = nextWayPt[1][0]
                    self.goalPose.position.z = nextWayPt[2][0]
                    
                    self.newWayPtReq = True
                
                # check if we are near the current pose and handle accordingly               
                if isItNear(self.mavrosPose, self.goalPose):
                   _ =  self.wayP2Nav.pop(0)
                   self.newWayPtReq = False
                
            else: 
                
                # path completed, we are ready for the next command
                self.currState_1 = STATE1_READY
        
        return
        
    def _main(self):
        
        # initialize system
        self._setup_mavROS()

        while(not rospy.is_shutdown()):
            
            if self.currState_0 == STATE0_IDLE:
                
                # check if self.mavrosState.mode is set to OFFBOARD and the agent is armed
                if self.mavrosState.mode == "OFFBOARD" and self.mavrosState.armed:
                    self.currState_0 = STATE0_ACTIVE                                       
                
            elif self.currState_0 == STATE0_ACTIVE:
                
                # TO DO
                self._runFSM()
                self._updatePose()
                
                # always publish pose
                if self.currState_1 != STATE1_IDLE:
                    self.pubPoseLocal.publish(self.goalPose)
                                    
            else:
                print("Nothing here")                    
            
            # sleep
            self.rate.sleep()
            
        return

if __name__ == "__main__":
    
    # create a roscoreDron object for task2
    droneT2 = roscoreDrone_T2('rob498_drone_04')
    droneT2._main()

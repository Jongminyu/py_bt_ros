from modules.base_bt_nodes import BTNodeList, Status, Node
from modules.base_bt_nodes import Sequence as BaseSequence
from modules.base_bt_nodes import Fallback, ReactiveSequence, ReactiveFallback, Parallel

from modules.base_bt_nodes_ros import ActionWithROSAction, ActionWithROSService

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus


class Sequence(BaseSequence):
    def __init__(self, node_type, children, name=None):
        actual_name = name if name else node_type
        super().__init__(actual_name, children)


class SaveHomePose(Node):
    def __init__(self, node_type, agent, name=None, topic_name="/amcl_pose"):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        
        self.agent = agent
        self.topic_name = topic_name 
        self.saved = False
        self.cache = None
        
        
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.agent.ros_bridge.node.create_subscription(
            PoseWithCovarianceStamped,   
            self.topic_name,
            self._callback,
            amcl_qos  
        )

    def _callback(self, msg):
        self.cache = msg

    async def run(self, agent, blackboard):
        if self.saved:
            return Status.SUCCESS

        if self.cache is not None:
            
            home_ps = PoseStamped()
            home_ps.header = self.cache.header
            home_ps.pose = self.cache.pose.pose
            
            blackboard['home_pose'] = home_ps
            self.agent.ros_bridge.node.get_logger().info(f"[{self.name}] Home Saved (from AMCL)")
            self.saved = True
            return Status.SUCCESS
        
        
        return Status.RUNNING


class WaitForGoalPose(Node):
    def __init__(self, node_type, agent, name=None, topic_name="/bt/goal_pose"):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.agent = agent
        self.topic_name = topic_name
        self.got_goal = False
        self.cache = None
        
        
        self.agent.ros_bridge.node.create_subscription(
            PoseStamped,
            self.topic_name,
            self._callback,
            10 
        )
        WaitForGoalPose.instance = self

    def _callback(self, msg):
        self.cache = msg

    async def run(self, agent, blackboard):
        if self.got_goal:
            return Status.SUCCESS
        if self.cache is not None:
            blackboard['target_pose'] = self.cache
            self.agent.ros_bridge.node.get_logger().info(f"[{self.name}] New Goal Received!")
            self.got_goal = True
            return Status.SUCCESS
        return Status.RUNNING


class NavigateToKey(ActionWithROSAction):
    def __init__(self, node_type, agent, bb_key_name, name=None):
        actual_name = name if name else node_type
        super().__init__(actual_name, agent, (NavigateToPose, 'navigate_to_pose'))
        self.bb_key_name = bb_key_name

    def _build_goal(self, agent, bb):
        target_ps = bb.get(self.bb_key_name)
        if target_ps is None:
            return None
        goal = NavigateToPose.Goal()
        goal.pose = target_ps
        return goal

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        return Status.FAILURE


class CaptureImageClient(ActionWithROSService):
    def __init__(self, node_type, agent, name=None):
        actual_name = name if name else node_type
        super().__init__(actual_name, agent, (Trigger, '/capture_image_service'))

    def _build_request(self, agent, blackboard):
        return Trigger.Request()
        
class ResetGoal(Node):
    def __init__(self, node_type, agent, name=None):
        actual_name = name if name else node_type
        super().__init__(actual_name)
        self.agent = agent

    async def run(self, agent, blackboard):
        w = getattr(WaitForGoalPose, "instance", None)
        if w is not None:
            w.got_goal = False
            w.cache = None

        
        if 'target_pose' in blackboard:
            del blackboard['target_pose']

        self.agent.ros_bridge.node.get_logger().info(
            f"[{self.name}] Goal reset. Waiting for next user command."
        )
        return Status.SUCCESS



BTNodeList.ACTION_NODES.extend([
    'SaveHomePose',
    'WaitForGoalPose',
    'NavigateToKey',
    'CaptureImageClient',
    'ResetGoal'
])

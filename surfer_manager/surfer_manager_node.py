
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from surfer_msgs.msg import Status
from surfer_msgs.srv import SetMode, Arming, SetBehavior, GetBehaviors, SetGroup

valid_modes = ['IDLE','MANUAL','AUTO']
valid_behaviors = ['CIRCLE','FOLLOW','WAYPOINT']

class SurferManager(Node):

    def __init__(self):
        super().__init__('mananger_node')
        # Define Class Variables
        self.i = 0
        self.status = Status()
        self.status_old = Status()
        self.status.mode = 'IDLE'


        self.declare_parameter('name','')
        self.declare_parameter('group','none')
        self.declare_parameter('id',0)
        self.declare_parameter('type','none')
        self.declare_parameter('behaviors',[''])

        self.status.name = self.get_parameter('name').get_parameter_value().string_value
        self.status.type = self.get_parameter('type').get_parameter_value().string_value
        self.status.id = self.get_parameter('id')._value
        self.status.group = self.get_parameter('group').get_parameter_value().string_value
        self.behaviors = self.get_parameter('behaviors').get_parameter_value().string_array_value
        self.name_msg = String()
        self.name_msg.data = self.status.name
        #self.status.name = str(self.get_parameters('name').get_parameter_value())
        #self.status.type = str(self.get_parameter('type').get_parameter_value())
        #self.status.group = str(self.get_parameter('group').get_parameter_value())

        self.pose = PoseWithCovarianceStamped()

        self.global_status_pub = self.create_publisher(Status, '/global_status', 10)
        self.status_pub = self.create_publisher(Status, 'status', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,'pose',10)


        self.odom_sub = self.create_subscription(Odometry,'odom',self.odomCallBack,10)

        self.set_mode_srv = self.create_service(SetMode,'set_mode',self.setModeCallBack)
        self.arming_srv = self.create_service(Arming,'arm',self.armCallBack)
        self.set_behave_srv = self.create_service(SetBehavior,'set_behavior',self.setBehaviorCallBack)
        self.get_behave_srv = self.create_service(GetBehaviors,'get_behaviors',self.getBehaviorsCallBack)
        self.set_group_srv = self.create_service(SetGroup,'set_group',self.setGroupCallBack)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.runLoop)


    def setModeCallBack(self,request,response):
        if request.mode in valid_modes:
            # Todo check to see if Mode request is valid and return True or False depending
            self.status.mode = request.mode
            response.res = True
        else:
            response.res = False

        return response


    def setBehaviorCallBack(self,request,response):
        if request.behavior in self.behaviors:
            # Todo check to see if Mode request is valid and return True or False depending
            self.status.behavior = request.behavior
            response.res = True
        else:
            response.res = False

        return response

    def getBehaviorsCallBack(self,request,response):
        response.behaviors = self.behaviors
        return response

    def setGroupCallBack(self,request,response):
        self.status.group = request.group
        if self.status.group == '':
            response.res = False

        else:
            response.res = True

        return response

    def armCallBack(self,request,response):
        # Todo Set conditions to allow arming. Need to be in water?
        # If we are disarmed and the request is to arm
        if (not self.status.armed and request.arm):
            self.status.armed = request.arm
            response.res = True # Response is the armed status

        if (self.status.armed and not request.arm):
            self.status.armed = request.arm
            response.res = False # Response is the armed status

        return response

    def odomCallBack(self,msg):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.pose = msg.pose

    def runLoop(self):
        self.pose_pub.publish(self.pose)        
        self.global_status_pub.publish(self.status)
        self.status_pub.publish(self.status)


def main(args=None):
    rclpy.init(args=args)

    my_surfer = SurferManager()

    rclpy.spin(my_surfer)
    my_surfer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#! /usr/bin/env python3
# Mofidied from Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import Spin, NavigateToPose
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

import tf2_geometry_msgs as tfg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

from sensor_msgs.msg import Image
from std_msgs.msg import String

import tf_transformations

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from playsound import playsound
from pydub import AudioSegment
from pydub.playback import play

from gtts import gTTS
from io import BytesIO
import pygame



class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)


qos_profile = amcl_pose_qos






class RobotCommander(Node):

    def __init__(self, node_name='robot_commander', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)


        self.pose_frame_id = 'map'

        # Flags and helper variables
        self.state = 0
        self.goal_handle = None
        self.cancel_goal = False
        self.result_future = None
        self.feedback = None
        self.status = None
        self.initial_pose_received = False
        self.is_docked = None

        self.last_destination_goal = ("go", (0.0, 0.0, 0.57))
        self.face_dist = 0.5
        self.ring_parking_dist = 0.3
        self.qr_parking_dist = 0.3
        self.navigation_list = []



        self.possible_rings = None
        self.curr_investigated_ring = None
        self.correct_painting = None

        self.seen_rings = {} # {barva: position np.array}
        self.seen_cylinders = {} # {barva: position np.array}
        self.seen_faces = [] # (position np.array;    slika ob pozdravljanju if isPainting else None).



        self.map_np = None
        self.map_data = {"map_load_time":None,
                         "resolution":None,
                         "width":None,
                         "height":None,
                         "origin":None} # origin will be in the format [x,y,theta]


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # # rc.point_hash(point) to point_in_map_frame. This way we don't duplicate points.
        # self.detected_faces = {}

        # self.face_marker_sub = self.create_subscription(Marker, "/people_marker", self.face_detect_callback, QoSReliabilityPolicy.BEST_EFFORT)


        # ROS2 subscribers
        self.occupancy_grid_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, qos_profile)

        self.create_subscription(DockStatus,
                                 'dock_status',
                                 self._dockCallback,
                                 qos_profile_sensor_data)

        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)


        self.face_sub = self.create_subscription(Marker, "/detected_faces", self.face_detected_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.cylinder_sub = self.create_subscription(Marker, "/detected_cylinder", self.cylinder_detected_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.ring_sub = self.create_subscription(Marker, "/detected_rings", self.ring_detected_callback, QoSReliabilityPolicy.BEST_EFFORT)

        # Here we abuse the Twist message because we don't want to go making our own one.
        self.top_img_stats_sub = self.create_subscription(Twist, "/top_img_stats", self.top_img_stats_callback, QoSReliabilityPolicy.BEST_EFFORT)
        self.latest_top_img_stats = (None, None, None) # (centroid, area, shape)
        self.top_img_changed = False

        self.bridge = CvBridge()
        self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.front_camera_img_saver, qos_profile_sensor_data)
        self.save_front_camera_img = False
        self.front_image_has_changed = False



        # ROS2 publishers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)

        # ROS2 Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')


        # for parking
        self.arm_pub = self.create_publisher(String, "/arm_command", QoSReliabilityPolicy.BEST_EFFORT)
        self.parking_pub = self.create_publisher(Twist, "cmd_vel", QoSReliabilityPolicy.BEST_EFFORT)

        # For waiting by spinning (this allows subscriptions to call callbacks, as opposed to time.sleep(0))
        self.waiting_spin_dir = 1 # vals 1 and -1


        self.get_logger().info(f"Robot commander has been initialized!")

    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        super().destroy_node()





    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


    def get_pose_obj(self, x, y, fi):

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = self.YawToQuaternion(fi)

        return goal_pose



    # Transformation functions:

    def map_pixel_to_world(self, x, y, theta=0):
        ### Convert a pixel in an numpy image, to a real world location
        ### Works only for theta=0
        assert not self.map_data["resolution"] is None

        # Apply resolution, change of origin, and translation
        #
        world_x = x*self.map_data["resolution"] + self.map_data["origin"][0]
        world_y = (self.map_data["height"]-y)*self.map_data["resolution"] + self.map_data["origin"][1]

        # Apply rotation
        return world_x, world_y

    def world_to_map_pixel(self, world_x, world_y, world_theta=0.2):
        ### Convert a real world location to a pixel in a numpy image
        ### Works only for theta=0
        assert self.map_data["resolution"] is not None

        # Apply resolution, change of origin, and translation
        # x is the first coordinate, which in opencv (numpy) that is the matrix row - vertical
        x = int((world_x - self.map_data["origin"][0])/self.map_data["resolution"])
        y = int(self.map_data["height"] - (world_y - self.map_data["origin"][1])/self.map_data["resolution"] )

        # Apply rotation
        return x, y


    # General RC stuff:


    def undock(self):
        """Perform Undock action."""
        self.info('Undocking...')
        self.undock_send_goal()

        while not self.isUndockComplete():
            time.sleep(0.1)

    def undock_send_goal(self):
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        goal_future = self.undock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.undock_goal_handle = goal_future.result()

        if not self.undock_goal_handle.accepted:
            self.error('Undock goal rejected')
            return

        self.undock_result_future = self.undock_goal_handle.get_result_async()

    def isUndockComplete(self):
        """
        Get status of Undock action.

        :return: ``True`` if undocked, ``False`` otherwise.
        """
        if self.undock_result_future is None or not self.undock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.undock_result_future, timeout_sec=0.1)

        if self.undock_result_future.result():
            self.undock_status = self.undock_result_future.result().status
            if self.undock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Undock succeeded')
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""

        # self.info('Checking if task is complete')
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(localizer)
        if not self.initial_pose_received:
            time.sleep(1)
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def YawToQuaternion(self, angle_z = 0.):
        quat_tf = quaternion_from_euler(0, 0, angle_z)

        # Convert a list to geometry_msgs.msg.Quaternion
        quat_msg = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        return quat_msg

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        self.current_pose = msg.pose
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def setInitialPose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = self.pose_frame_id
        msg.header.stamp = 0
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return



















    # Our code:


    # Somewhat general functions:

    def rgb2hsv(self, r, g, b):

        c_high = max(r, max(g, b))
        c_low = min(r, min(g, b))
        c_rng = c_high - c_low

        s = 0
        if c_high > 0:
            s = c_rng / c_high

        v = c_high / 255

        r1 = (c_high - r) / c_rng
        g1 = (c_high - g) / c_rng
        b1 = (c_high - b) / c_rng

        h1 = 0
        if r == c_high:
            h1 = b1 - g1
        elif g == c_high:
            h1 = r1 - b1 + 2
        elif b == c_high:
            h1 = g1 - r1 + 4

        h = 0
        if c_rng == 0:
            h = 0
        elif h1 < 0:
            h = (h1 + 6) / 6
        else:
            h = h1 / 6

        h = h * 360

        return h, s, v


    def create_marker(self, point_stamped, marker_id):
        """You can see the description of the Marker message here: https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html"""
        marker = Marker()

        marker.header = point_stamped.header

        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.id = marker_id

        # Set the scale of the marker
        scale = 0.15
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z

        return marker

    def make_cv2_window(self):
        cv2.namedWindow("Just for showing what is in rc.map_np", cv2.WINDOW_NORMAL)

    def show_map(self):
        cv2.imshow("Just for showing what is in rc.map_np", self.map_np)
        cv2.waitKey(1000)

    def get_colour_str_from_hue_and_val(self, h, v):

        color = ""
        if(v < 0.1):
            color = "black"
        elif h < 15 or h > 350:
            color = "red"
        elif h > 20 and h < 65:
            color = "yellow"
        elif h > 65 and h < 150:
            color = "green"
        elif h > 180 and h < 265:
            color = "blue"
        
        return color



    # Various permanent setters:



    def set_top_camera(self, pos_str="normal"):

        sending_str = ""
        if pos_str == "normal":
            sending_str = "garage"
        elif pos_str == "park":
            sending_str = "look_for_parking"
        elif pos_str == "qr":
            sending_str = "look_for_qr"

        msg = String()
        msg.data = sending_str
        self.arm_pub.publish(msg)

    def set_state(self, state, params_dict=None):

        if state == 1:
            self.state = 1
            self.possible_rings = params_dict["ring_list"]

            if self.possible_rings[0] in self.seen_rings.keys():
                ring_loc, ring_col = self.seen_rings[self.possible_rings[0]]
                self.QR_code_sequence(ring_loc, ring_col)
            elif self.possible_rings[1] in self.seen_rings.keys():
                ring_loc, ring_col = self.seen_rings[self.possible_rings[1]]
                self.QR_code_sequence(ring_loc, ring_col)
            
            print("self.possible_rings:")
            print(self.possible_rings)


        elif state == 2:
            self.state = 2

            new_ring_list = params_dict["ring_list"]
            # if it has two elements, we get their intersection (came from person answer)
            # if it has one element, that one is wrong (came from wrong QR code)
            if len(new_ring_list) == 2:
                self.possible_rings = list(set(self.possible_rings) & set(new_ring_list))
            elif len(new_ring_list) == 1:
                self.possible_rings = list(set(self.possible_rings) - set(new_ring_list))

            if self.possible_rings[0] in self.seen_rings.keys():
                ring_loc, ring_col = self.seen_rings[self.possible_rings[0]]
                self.QR_code_sequence(ring_loc, ring_col)
            
            print("self.possible_rings:")
            print(self.possible_rings)

        elif state == 3:
            self.state = 3
            
            painting_ixs = []
            for ix, paint in self.seen_faces:
                if not paint is None:
                    painting_ixs.append(ix)

            # show anomalies for existing paintings
            

        elif state == 4:
            self.state = 4
        elif state == 5:
            self.state = 5



    # Callbacks and their supporting functions:


    def map_callback(self, msg):
            self.get_logger().info(f"Read a new Map (Occupancy grid) from the topic.")
            # reshape the message vector back into a map
            self.map_np = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            # fix the direction of Y (origin at top for OpenCV, origin at bottom for ROS2)
            self.map_np = np.flipud(self.map_np)
            # change the colors so they match with the .pgm image
            self.map_np[self.map_np==0] = 127
            self.map_np[self.map_np==100] = 0
            # load the map parameters
            self.map_data["map_load_time"]=msg.info.map_load_time
            self.map_data["resolution"]=msg.info.resolution
            self.map_data["width"]=msg.info.width
            self.map_data["height"]=msg.info.height
            quat_list = [msg.info.origin.orientation.x,
                        msg.info.origin.orientation.y,
                        msg.info.origin.orientation.z,
                        msg.info.origin.orientation.w]
            self.map_data["origin"]=[msg.info.origin.position.x,
                                    msg.info.origin.position.y,
                                    tf_transformations.euler_from_quaternion(quat_list)[-1]]
            #self.get_logger().info(f"Read a new Map (Occupancy grid) from the topic.")



    def face_detected_callback(self, msg):

        if self.state == 5:
            return

        self.info("Face detected!")

        face_location = np.array([msg.pose.position.x, msg.pose.position.y])

        self.seen_faces.append((face_location, None))

        
        add_to_navigation = self.get_nav_goals_to_position(face_location, self.face_dist)
        
        add_to_navigation.extend([
            ("face_or_painting", 0),
            ("spin", 3.14),
            self.last_destination_goal
        ])

        self.prepend_to_nav_list(add_to_navigation, spin_full_after_go=False)

        self.cancel_goal = True
        # self.cancelTask()


    def ring_detected_callback(self, msg):

        if self.state > 2:
            return

        
        self.info("Ring detected!")

        ring_location = np.array([msg.pose.position.x, msg.pose.position.y])
        
        r = int(msg.color.r * 255)
        g = int(msg.color.g * 255)
        b = int(msg.color.b * 255)

        h, s, v = self.rgb2hsv(r, g, b)

        color = self.get_colour_str_from_hue_and_val(h, v)

        self.seen_rings[color] = ring_location


        if self.possible_rings is None:
            return
        
        if color in self.possible_rings:
            self.QR_code_sequence(ring_location, color)



    def cylinder_detected_callback(self, msg):

        self.info("Cylinder detected!")

        r = int(msg.color.r * 255)
        g = int(msg.color.g * 255)
        b = int(msg.color.b * 255)

        h, s, v = self.rgb2hsv(r, g, b)

        color = self.get_colour_str_from_hue_and_val(h, v)

        self.seen_cylinders[color] = np.array([msg.pose.position.x, msg.pose.position.y])



    def top_img_stats_callback(self, msg):

        # We decided to abuse the twist message so we don't have to make our own one.

        # The ordering is the same that we get from get_area_and_centroid() and img.shape in image_gatherer.py
        # Names msg.linear.x don't necessarily mean the x coodrinate. It is simply the first of the two components.

        centroid = (msg.linear.x, msg.linear.y)
        area = msg.linear.z
        shape = (int(msg.angular.x), int(msg.angular.y))

        self.latest_top_img_stats = (centroid, area, shape)
        self.top_img_changed = True


    def get_latest_top_img_stats(self):
        top_img_has_changed = self.top_img_changed
        self.top_img_changed = False
        return self.latest_top_img_stats[0], self.latest_top_img_stats[1], self.latest_top_img_stats[2], top_img_has_changed


    def get_top_img_stats_with_waiting_for_change(self):

        centroid, area, shape, top_img_has_changed = self.get_latest_top_img_stats()

        is_none_present = area is None or centroid is None or shape is None

        cycle_duration = 1000    # miliseconds
        cycle_start_time = time.time()
        while is_none_present or not top_img_has_changed:

            if (time.time() - cycle_start_time) >= cycle_duration:
                cycle_start_time = time.time()
                print("Waiting for new image stats...")

            self.wait_by_spinning()
            centroid, area, shape, top_img_has_changed = self.get_latest_top_img_stats()



        return centroid, area, shape


    def front_camera_img_saver(self, data):

        if True or self.save_front_camera_img:
            try:
                self.front_image = self.bridge.imgmsg_to_cv2(data, "bgr8").copy()
                
                # cv2.imshow("Front camera image Original", self.front_image)
                # key = cv2.waitKey(1)
                # if key==27:
                #     print("exiting")
                #     exit()

                self.front_image_has_changed = True
            except CvBridgeError as e:
                print(e)


    # Questioning and paintings:


    def get_red_pixels_thresholded(self, img):


        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


        # hue_img = hsv_img[:,:,0]

        # hue_low_treshold = 50
        # hue_low_img =  hue_img > hue_low_treshold

        # hue_high_treshold = 70
        # hue_high_img =  hue_img < hue_high_treshold




        red_img = img[:,:,2]

        red_low_treshold = 30
        red_low_img = red_img > red_low_treshold

        # red_high_treshold = 70
        # red_high_img = red_img < red_high_treshold



        green_img = img[:,:,1]

        green_high_treshold = 7
        green_img = green_img < green_high_treshold


        blue_img = img[:,:,0]

        blue_high_treshold = 7
        blue_img = blue_img < blue_high_treshold



        # val_img = hsv_img[:,:,2]
        
        # val_low_treshold = 10
        # val_img = val_img > val_low_treshold

        colour_tresholded = np.logical_and(red_low_img, green_img)
        temp_list = [blue_img] #, val_img]
        for t in temp_list:
            colour_tresholded = np.logical_and(colour_tresholded, t)

        # colour_tresholded = np.logical_and(hue_low_img, hue_high_img, val_img)
        # colour_tresholded = np.logical_and(red_low_img, green_img, blue_img, val_img)
        
        colour_tresholded = colour_tresholded.astype(np.uint8)

        return colour_tresholded

    def get_answer(self):
        # self.say_question()
        print("Question not said, because gtts is not working.")
        self.handle_speech_to_text()
        
        

    def handle_speech_to_text(self):
        allowed_answers = ["black", "red", "yellow", "green", "blue"]
        answers = []
        while len(answers) < 2:
            answer = input("Enter the color of the painting: ")
            if answer in allowed_answers:
                answers.append(answer)
            else:
                print("Invalid answer. Please try again.")
                print("Allowed answers are: ", allowed_answers)
        
        if self.state == 0:
            self.set_state(1, {"ring_list": answers})
        elif self.state == 1:
            self.set_state(2, {"ring_list": answers})


    def face_or_painting(self):

        self.save_front_camera_img = True

        while not self.front_image_has_changed:
            self.wait_by_spinning()
        
        curr_img = self.front_image
        self.save_front_camera_img = False
        self.front_image_has_changed = False

        cv2.imshow("Front camera image", curr_img)
        key = cv2.waitKey(1)
        if key==27:
            print("exiting")
            exit()
        
        red_frame = self.get_red_pixels_thresholded(curr_img)

        red_frame_to_show = curr_img.copy()
        for i in range(3):
            red_frame_to_show[:,:,i] = red_frame_to_show[:,:,i] * red_frame
        cv2.imshow("Red frame", red_frame_to_show)
        key = cv2.waitKey(1)
        if key==27:
            print("exiting")
            exit()


        # If less than 10 pixels are red, we assume that there is no frame.
        if red_frame.sum() < 100:
            print("No frame found!")
            if self.state <= 1:
                self.get_answer()
            return
        



        # Initial try:
        """
        contours, _ = cv2.findContours(red_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour which will correspond to the red frame
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)


            cropped_image = curr_img[y:y+h, x:x+w]


            # Example: Convert to grayscale and apply a binary threshold
            gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
            _, thresholded_image = cv2.threshold(gray_image, 128, 255, cv2.THRESH_BINARY)

            cv2.imshow("Tresholded img:", thresholded_image)
            key = cv2.waitKey(1)
            if key==27:
                print("exiting")
                exit()"""



        # Show all child contours - doesn't work:
        """
        image = red_frame_to_show.copy()


        # Assuming `red_frame` is a binary mask of the red areas
        contours, hierarchy = cv2.findContours(red_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through the contour and hierarchy list
        for i in range(len(contours)):
            # Check if the contour has a parent, indicating it is inside another contour
            if hierarchy[0][i][3] != -1:
                inner_contour = contours[i]
                cv2.drawContours(image, [inner_contour], -1, (0, 255, 0), 2)
        
        while True:
            cv2.imshow("Inner contours", image)
            key = cv2.waitKey(1)
            if key==27:
                print("exiting")
                exit()"""



        # Trying to combine the approaches:
        """
        image = red_frame.copy()
        image[:,:] = 0

        # Assuming 'image' is your preprocessed image (grayscale and thresholded)
        contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            print("No contours found!" + 5*"\n")
            contours, _ = cv2.findContours(red_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                print("Num of new contours: ", len(contours))
                # Find the largest contour which will correspond to the red frame

                # This sometimes finds some specs. But interestingly, the frame seems to always be just one contour.
                contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(contour)
            
        elif len(contours) == 1:
            print("Only one contour found!" + 5*"\n")

            contour = contours[0]
            x, y, w, h = cv2.boundingRect(contour)
            
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)

            cv2.imshow("Inner contours", image)
            key = cv2.waitKey(1)
            if key==27:
                print("exiting")
                exit()


        else:

            hierarchy = hierarchy[0]  # Get the first item in hierarchy structure

            innermost_contours = []
            for i, (contour, hier) in enumerate(zip(contours, hierarchy)):
                # Check if the contour has no children and has a parent
                
                cv2.imshow("Contour", image)
                key = cv2.waitKey(1)
                if key==27:
                    print("exiting")
                    exit()
                input("Press Enter to continue...")

                if hier[2] < 0 and hier[3] != -1:
                    innermost_contours.append(contour)

            if len(innermost_contours) > 1:
                print("More than one innermost contour found!" + 5*"\n")
            
            if len(innermost_contours) == 0:
                print("No innermost contour found!" + 5*"\n")
                return

            for contour in innermost_contours:
                cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
            
            contour = innermost_contours[0]

            
            while True:
                cv2.imshow("Inner contours", image)
                key = cv2.waitKey(1)
                if key==27:
                    print("exiting")
                    exit()



        x, y, w, h = cv2.boundingRect(contour)

        # Example points (you need to define these based on your specific case)
        src_points = np.array([[x, y], [x+w, y], [x, y+h], [x+w, y+h]], dtype='float32')

        dst_points = np.array([[0, 0], [w, 0], [0, h], [w, h]], dtype='float32')  # Destination points

        # Compute the homography matrix
        H, _ = cv2.findHomography(src_points, dst_points)

        # paint_w, paint_h = 240, 320
        # transformed_image = cv2.warpPerspective(curr_img, H, (curr_img.shape[1], curr_img.shape[0]))
        transformed_image = cv2.warpPerspective(curr_img, H, (w, h))

        cv2.imshow("Homography:", transformed_image)
        key = cv2.waitKey(1)
        if key==27:
            print("exiting")
            exit()


        """




        image = red_frame.copy()
        image[:,:] = 0

        contours, _ = cv2.findContours(red_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            print("Num of new contours: ", len(contours))
            # Find the largest contour which will correspond to the red frame

            # This sometimes finds some specs.
            # But interestingly, the frame seems to always be just one contour, so this works.
            contour = max(contours, key=cv2.contourArea)

            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx_corners = cv2.approxPolyDP(contour, epsilon, True)

            # a corner is of the form [[x, y]] for some reason
            # Returning tuple from lambda apparently has it sorted by x and then by y
            approx_corners = sorted(approx_corners, key=lambda x: (x[0][0], x[0][1]))
            approx_corners = np.array(approx_corners)


            if len(approx_corners) == 4:  # Check if the contour can be approximated to four points
                src_points = approx_corners.reshape(4, 2).astype('float32')
            else:
                print("The contour cannot be approximated to four corners.")
                return
        else:
            print("No contours found!" + 5*"\n")
            return


        print("approx_corners:")
        print(approx_corners)
        print("src_points:")
        print(src_points)


        # # Only gets corners of the bounding box
        # x, y, w, h = cv2.boundingRect(contour)
        # src_points = np.array([[x, y], [x+w, y], [x, y+h], [x+w, y+h]], dtype='float32')

        # print("curr_img.shape[0]")
        # print(curr_img.shape[0])

        # width = 320# curr_img.shape[1]
        # height = 240 #curr_img.shape[0]

        # width = curr_img.shape[1]
        # height = curr_img.shape[0]


        width = 444
        height = 648


        # dst_points = np.array([[0, 0], [width, 0], [0, height], [width, height]], dtype='float32')  # Destination points
        dst_points = np.array([[0, 0], [0, height], [width, 0], [width, height]], dtype='float32')  # Destination points

        # Compute the homography matrix
        H, _ = cv2.findHomography(src_points, dst_points)


        # paint_w, paint_h = 240, 320
        # transformed_image = cv2.warpPerspective(curr_img, H, (curr_img.shape[1], curr_img.shape[0]))
        transformed_image = cv2.warpPerspective(curr_img, H, (width, height))

        repeat = True
        while repeat:

            for i in range(1000):
                cv2.imshow("Homography:", transformed_image)
                key = cv2.waitKey(1)
                if key==27:
                    print("exiting")
                    exit()
            
            curr_str = input("Write 'done' to continue...")
            if curr_str == "done":
                repeat = False










    # Basic navigation:


    def get_curr_pos(self):
            # Create a PointStamped in the /base_link frame of the robot
            # The point is located 0.5m in from of the robot
            # "Stamped" means that the message type contains a Header
            point_in_robot_frame = PointStamped()
            point_in_robot_frame.header.frame_id = "/base_link"
            point_in_robot_frame.header.stamp = self.get_clock().now().to_msg()

            point_in_robot_frame.point.x = 0.
            point_in_robot_frame.point.y = 0.
            point_in_robot_frame.point.z = 0.

            # Now we look up the transform between the base_link and the map frames
            # and then we apply it to our PointStamped


            time_now = rclpy.time.Time()
            timeout =rclpyDuration(seconds=0.1)
            try:
                # An example of how you can get a transform from /base_link frame to the /map frame
                # as it is at time_now, wait for timeout for it to become available
                trans = self.tf_buffer.lookup_transform("map", "base_link", time_now, timeout)
                self.get_logger().info(f"Looks like the transform is available.")

                # Now we apply the transform to transform the point_in_robot_frame to the map frame
                # The header in the result will be copied from the Header of the transform
                point_in_map_frame = tfg.do_transform_point(point_in_robot_frame, trans)
                self.get_logger().info(f"We transformed a PointStamped!")

                return point_in_map_frame


            except TransformException as te:
                self.get_logger().info(f"Cound not get the transform: {te}")


    def get_curr_pos_with_waiting(self):
        curr_pos = self.get_curr_pos()
        while curr_pos is None:
            print("Waiting for point...")
            time.sleep(0)
            curr_pos = self.get_curr_pos()
        return curr_pos


    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)

        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def correct_orientation(self, goal_location, behavior_tree=''):
        
        # This was supposed to make things better, but it doesn't do much, and it
        # disables you using keyboards to help the robot.
        return
        
        # Simply spin towards the goal location. Meant for problems with orientation.
        curr_pos = self.get_curr_pos_with_waiting()

        curr_pos_location = np.array([curr_pos.point.x, curr_pos.point.y])

        vec_to_goal_normed = goal_location - curr_pos_location

        fi = np.arctan2(vec_to_goal_normed[1], vec_to_goal_normed[0])

        pose = self.get_pose_obj(curr_pos_location[0],curr_pos_location[1], fi)

        self.goToPose(pose)




    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return


    def spin(self, spin_dist=1.57, time_allowance=10, print_info=True):
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)

        if print_info:
            self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def add_tup_to_nav_list(self, tup, spin_full_after_go=False):

        if tup[0] == "go":
            self.navigation_list.append(("go", self.get_pose_obj(*tup[1]), tup[1]))
            if spin_full_after_go:
                self.navigation_list.append(("spin", 6.28, None))
        elif tup[0] == "spin":
            self.navigation_list.append(("spin", tup[1], None))
        elif tup[0] == "face_or_painting":
            self.navigation_list.append(("face_or_painting", None, None))
        elif tup[0] == "say_color":
            self.navigation_list.append(("say_color", tup[1], None))
        elif tup[0] == "park":
            self.navigation_list.append(("park", None, None))
        elif tup[0] == "read_qr":
            self.navigation_list.append((("read_qr", None, None)))
        elif tup[0] == "correct_orientation":
            self.navigation_list.append(("correct_orientation", tup[1], None))

    def add_to_nav_list(self, to_add_list, spin_full_after_go=False):

        for tup in to_add_list:
            self.add_tup_to_nav_list(tup, spin_full_after_go)
    
    def prepend_tup_to_nav_list(self, tup, spin_full_after_go=False, insert_pos=0):

        if tup[0] == "go":
            self.navigation_list.insert(insert_pos, ("go", self.get_pose_obj(*tup[1]), tup[1]))
            if spin_full_after_go:
                self.navigation_list.insert(insert_pos, ("spin", 6.28, None))
        elif tup[0] == "spin":
            self.navigation_list.insert(insert_pos, ("spin", tup[1], None))
        elif tup[0] == "face_or_painting":
            self.navigation_list.insert(insert_pos, ("face_or_painting", None, None))
        elif tup[0] == "say_color":
            self.navigation_list.insert(insert_pos, ("say_color", tup[1], None))
        elif tup[0] == "park":
            self.navigation_list.insert(insert_pos, ("park", None, None))
        elif tup[0] == "read_qr":
            self.navigation_list.insert(insert_pos, ("read_qr", None, None))
        elif tup[0] == "correct_orientation":
            self.navigation_list.insert(insert_pos, ("correct_orientation", tup[1], None))

    def prepend_to_nav_list(self, to_add_list, spin_full_after_go=False):
        
        # This makes thigs too complicated:
        """
        # inserting instead of prepending if we are already going to a face.
        # To aleviate at least some of the chaos that is caused by this.
        if len(to_add_list) >=2:
            if to_add_list[1][0] == "face_or_painting":
                
                insert_pos = 0
                face_or_painting_goal_dist = 4 # this is now 4, because we added the correction of orientation goal.

                window = [i[0] for i in self.navigation_list[:face_or_painting_goal_dist]]
                if "face_or_painting" in window:
                    insert_pos = window.index("face_or_painting")
                # If face_or_painting isn't near, add this action to the start.
                else:
                    insert_pos = 0
                    for tup in reversed(to_add_list):
                        self.prepend_tup_to_nav_list(tup, spin_full_after_go, insert_pos)
                    return
                
                # Find the last face_or_painting 
                next_insert_pos = insert_pos + face_or_painting_goal_dist
                while self.navigation_list[next_insert_pos][0] == "face_or_painting":
                    insert_pos = next_insert_pos
                    next_insert_pos += face_or_painting_goal_dist
                
                # now we have insert_pos at last face_or_painting
                # Add 1 to add after spin.
                insert_pos += 1
                
                # This is the self.last_destination_goal. 
                # We don't want it, because it is already given by the previous face_or_painting.
                del to_add_list[3]
                for tup in reversed(to_add_list):
                    self.prepend_tup_to_nav_list(tup, spin_full_after_go, insert_pos)
                return
        """

        for tup in reversed(to_add_list):
            self.prepend_tup_to_nav_list(tup, spin_full_after_go)
            

    def get_nav_goals_to_position(self, goal_location, parking_dist=0.5):
        
        print("goal_location:")
        print(goal_location)

        curr_pos = self.get_curr_pos_with_waiting()

        curr_pos_location = np.array([curr_pos.point.x, curr_pos.point.y])

        vec_to_goal_normed = goal_location - curr_pos_location
        vec_to_goal_normed /= np.linalg.norm(vec_to_goal_normed)

        face_goal_location = goal_location - parking_dist * vec_to_goal_normed

        fi = np.arctan2(vec_to_goal_normed[1], vec_to_goal_normed[0])

        nav_goals = [
            ("go", (face_goal_location[0], face_goal_location[1], fi)),
            ("correct_orientation", (goal_location[0], goal_location[1])),
            ]

        return nav_goals


    # Parking:

    def get_parking_navigation_goals(self, ring_location):

        self.set_top_camera("park")
        
        navigation_goals = self.get_nav_goals_to_position(ring_location, self.ring_parking_dist)

        navigation_goals.append(("park", None))

        return navigation_goals

    def wait_by_spinning(self, spin_dist=2*3.14 * 1e-7, spin_seconds=1):
        # Keep spin seconds low so that we remain in almost the same place.
        # Spin seconds has to be int, so instead we keep the distance low.
        spin_dir = self.waiting_spin_dir
        self.waiting_spin_dir = -self.waiting_spin_dir

        self.spin(spin_dir * spin_dist, spin_seconds, print_info=False)

    def cmd_vel(self, direction, miliseconds, linear_velocity=70.0, angular_velocity=1.5):
        # Directions: "forward", "right", "left", "backward"
        # miliseconds: how long to move

        velocity = Twist()

        if direction == "forward":
            velocity.linear.x = linear_velocity
        elif direction == "backward":
            velocity.linear.x = -linear_velocity
        elif direction == "left":
            velocity.angular.z = -angular_velocity
        elif direction == "right":
            velocity.angular.z = angular_velocity


        self.parking_pub.publish(velocity)


        # I think the velocity persists if you don't reset it.

        duration = miliseconds/1000 # to get seconds

        start_time = time.time()
        while (time.time() - start_time) < duration:
            time.sleep(0)

        velocity = Twist()
        self.parking_pub.publish(velocity)

    def park(self):

        self.set_top_camera("park")

        # Just here to wait until we get the first image, if that hasn't happened yet.
        _, _, _ = self.get_top_img_stats_with_waiting_for_change()

        angles = []
        areas_at_angles = []


        point_in_map_frame = self.get_curr_pos()
        while point_in_map_frame is None:
            print("Waiting for point...")
            time.sleep(0)
            point_in_map_frame = self.get_curr_pos()

        x = point_in_map_frame.point.x
        y = point_in_map_frame.point.y

        num_of_angles = 8
        for i in range(num_of_angles):
            fi = i * 2*3.14 / num_of_angles
            pose = self.get_pose_obj(x, y, fi)
            self.goToPose(pose)

            while not self.isTaskComplete():
                time.sleep(0)

            _, area, _ = self.get_top_img_stats_with_waiting_for_change()

            angles.append(fi)
            areas_at_angles.append(area)


        max_area_ix = areas_at_angles.index(max(areas_at_angles))
        chosen_fi = angles[max_area_ix]

        pose = self.get_pose_obj(x, y, chosen_fi)
        self.goToPose(pose)

        while not self.isTaskComplete():
            time.sleep(0)



        acceptable_errors = [5, 3]
        acceptable_areas = [5000, 1500]

        for i in range(len(acceptable_errors)):
            self.top_camera_centre_robot_to_blob_centre(acceptable_error=acceptable_errors[i], milliseconds=100, printout=True)
            self.top_camera_reduce_blob_area(acceptable_area=acceptable_areas[i], milliseconds=100, printout=True)



        print("Parking complete!")

    def top_camera_centre_robot_to_blob_centre(self, acceptable_error=10, milliseconds=15, printout=False):

        centroid, _, shape = self.get_top_img_stats_with_waiting_for_change()

        img_middle_x = shape[1] / 2

        while not(np.abs(centroid[0] - img_middle_x) < acceptable_error):

            if(printout):
                print("Centroid[0]: ", centroid[0])
                print("img_middle_x: ", img_middle_x)
                print("np.abs(centroid[0] - img_middle_x):")
                print(np.abs(centroid[0] - img_middle_x))

            if centroid[0] < img_middle_x:
                self.cmd_vel("right", milliseconds)
            else:
                self.cmd_vel("left", milliseconds)

            centroid, _, shape = self.get_top_img_stats_with_waiting_for_change()

    def top_camera_reduce_blob_area(self, acceptable_area=100, milliseconds=1, printout=False):

        _, area, _ = self.get_top_img_stats_with_waiting_for_change()

        while area > acceptable_area:
            if(printout):
                print("Area: ", area)

            self.cmd_vel("forward", milliseconds)
            _, area, _ = self.get_top_img_stats_with_waiting_for_change()

    
    # QR code reading:


    def QR_code_sequence(self, ring_location, ring_color):
        self.curr_investigated_ring = ring_color
        nav_goals = self.get_parking_navigation_goals(ring_location)
        nav_goals.append(self.get_read_near_cylinder_qr_goals())
        nav_goals.append(("spin", 3.14))
        nav_goals.append(self.last_destination_goal)

        self.prepend_to_nav_list(nav_goals, spin_full_after_go=False)
        self.cancel_goal = True

        return nav_goals
    
    def get_read_near_cylinder_qr_goals(self):
        self.set_top_camera("qr")

        curr_pos = self.get_curr_pos_with_waiting()
        curr_pos_location = np.array([curr_pos.point.x, curr_pos.point.y])
        
        nearest_pos = None
        for pos in self.seen_cylinders.values():
            if nearest_pos is None:
                nearest_pos = pos
            elif np.linalg.norm(pos - curr_pos_location) < np.linalg.norm(nearest_pos - curr_pos_location):
                nearest_pos = pos
        
        nav_goals = self.get_nav_goals_to_position(nearest_pos, self.qr_parking_dist)
        nav_goals.append(("read_qr", None))

        return nav_goals
    
    def read_qr(self):

        self.info("Reading QR code near the cylinder.")

        print("QR reading not implemented!")

        poskus_uspel = False
        dobljen_paining = None

        if poskus_uspel:
            # # Zakomentirano ze obdela set_state
            # if self.curr_investigated_ring == self.possible_rings[0]:
            #     del self.possible_rings[0]
            # elif self.curr_investigated_ring == self.possible_rings[1]:
            #     del self.possible_rings[1]
            self.set_state(2, {"ring_list": [self.curr_investigated_ring]})
        else:
            self.correct_painting = dobljen_paining
            self.set_state(3)
        
        self.info("QR code read.")
        self.set_top_camera("park")


    # Speech related:

    def say(self, what_to_say):

        mp3_fp = BytesIO()
        tts = gTTS(what_to_say, lang="en")
        tts.write_to_fp(mp3_fp)
        mp3_fp.seek(0)

        pygame.init()
        pygame.mixer.init()
        pygame.mixer.music.load(mp3_fp)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)

        #Če Pygame ne dela:
        #tts.save("color.mp3")
        #playsound("/color.mp3")

        #Če Playsound ne dela:
        #v = vlc.MediaPlayer("/color.mp3")
        #v.play()


    def say_question(self):

        question = "Do you know what color the ring is?"
        self.say(question)

    def say_color(self, color: str):

        self.info(color)
        self.say(color)






def main(args=None):

    rclpy.init(args=args)

    rc = RobotCommander()

    # Wait until Nav2 and Localizer are available
    rc.waitUntilNav2Active()


    rc.set_top_camera("park")

    # Check if the robot is docked, only continue when a message is recieved
    while rc.is_docked is None:
        rclpy.spin_once(rc, timeout_sec=0.5)

    # If it is docked, undock it first
    if rc.is_docked:
        rc.undock()











    # The mesh in rviz is coordinates.
    # The docking station is 0,0
    # Use Publish Point to hover and see the coordinates.
    # x: -2 to 4
    # y: -2.5 to 5


    # contains tuples of three types:
    # ("go", <PoseStamped object>), ("spin", angle_to_spin_by), ("face_or_painting", None)


    # UP = 0.0
    # RIGHT = 1.57
    # DOWN = 3.14
    # LEFT = 4.71

    UP = 0.0
    LEFT = 1.57
    DOWN = 3.14
    RIGHT = 4.71

    add_to_navigation = [

        ("spin", 3.14),

        ("go", (-0.65, 0., DOWN)),

        ("go", (1.1, -2., UP)),

        ("go", (3.5, -1.3, DOWN)),

        ("go", (2.8, -0.8, LEFT)),

        ("go", (2.35, 1.35, RIGHT)),

        ("go", (1.6, 0., DOWN)),

        ("go", (0.95, 1.75, DOWN)),

        ("go", (-1.08, 0.92, DOWN)),

        ("go", (-1.35, 3.35, RIGHT)),

        ("go", (0.5, 3.2, UP)),

        ("go", (2.15, 1.8, RIGHT)),

    ]

    rc.add_to_nav_list(add_to_navigation, spin_full_after_go=False)
    rc.add_to_nav_list(add_to_navigation, spin_full_after_go=True)



    while len(rc.navigation_list) > 0:


        print("\n\n")
        print(rc.navigation_list[0][0])
        print(rc.navigation_list[0][2])
        print("Goals following it:")

        desired_num_of_following_goals = 10
        for i in range(1, desired_num_of_following_goals+1):
            if i < len(rc.navigation_list):
                print(rc.navigation_list[i][0])
                print(rc.navigation_list[i][2])
                if i == desired_num_of_following_goals:
                    print("And more...")

        print("\n\n")

        print("rc.state:")
        print(rc.state)
        print("rc.possible_rings:")
        print(rc.possible_rings)
        print("rc.seen_cylinders:")
        print(rc.seen_cylinders)
        print("rc.seen_rings:")
        print(rc.seen_rings)

        print(4*"\n")





        curr_type, curr_goal, curr_goal_coordinates = rc.navigation_list[0]

        if curr_type == "go":
            
            # Attempt at making things more efficient,but it becomes more complicated.
            # # Saving last destination, if it was part of the general course.
            # if len(rc.navigation_list) >= 2:  
            #     next_type = rc.navigation_list[1][0]
            #     if next_type != "face_or_painting" and next_type != "park":
            #         rc.last_destination_goal = (curr_type, curr_goal_coordinates)
            
            rc.last_destination_goal = (curr_type, curr_goal_coordinates)

    
            rc.goToPose(curr_goal)

        elif curr_type == "spin":
            rc.spin(curr_goal)

        elif curr_type == "face_or_painting":
            rc.face_or_painting()

        elif curr_type == "say_color":
            rc.say_color(curr_goal)

        elif curr_type == "park":
            rc.set_top_camera("park")
            rc.park()

        elif curr_type == "read_qr":
            rc.read_qr()

        elif curr_type == "correct_orientation":
            rc.correct_orientation(curr_goal)

        del rc.navigation_list[0]



        printout_counter = 0
        while not rc.isTaskComplete():

            # if mg.clicked or rc.stop_spin:
            #     rc.cancel_goal()
            if rc.cancel_goal:
                rc.cancel_goal = False
                rc.cancelTask()

            if printout_counter % 3 == 0:
                rc.info("Waiting for the task to complete...")
            printout_counter += 1

            time.sleep(1)




        # input("Enter sth to continue.")


    input("Navigation list completed, waiting to terminate. Enter anything.")


    """# Example
    if False:
        # Finally send it a goal to reach
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rc.get_clock().now().to_msg()

        goal_pose.pose.position.x = 2.6
        goal_pose.pose.position.y = -1.3
        goal_pose.pose.orientation = rc.YawToQuaternion(0.57)

        rc.goToPose(goal_pose)

        while not rc.isTaskComplete():
            rc.info("Waiting for the task to complete...")
            time.sleep(1)

        rc.spin(-0.57)"""


    rc.destroyNode()

    # And a simple example
if __name__=="__main__":
    main()
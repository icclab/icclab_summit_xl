import rospy
import numpy as np
import copy
import tf
import datetime
import gc
from tools import *
from pprint import pprint
from pyquaternion import Quaternion
from gpd.msg import GraspConfigList
from moveit_python import *
from moveit_msgs.msg import Grasp, PlaceLocation
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from moveit_python.geometry import rotate_pose_msg_by_euler_angles, translate_pose_msg
from tf.transformations import *
import geometry_msgs.msg #for pose2 simple
import math
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.msg import tfMessage
import time
from send_gripper import open_gripper_summit, close_gripper_summit

from tf import TransformListener
import copy
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import sensor_msgs
import actionlib
from filter_pointcloud_client import call_pointcloud_filter_service
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint, JointConstraint
from copy import deepcopy
from pointcloud_operations import create_mesh_and_save
from sensor_msgs import point_cloud2
from std_srvs.srv import Empty

client = None
from moveit_msgs.msg import MoveItErrorCodes

# Build a useful mapping from MoveIt error codes to error names
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class GpdPickPlace(object):
    grasps = []
    mark_pose = False
    grasp_offset = -0.08
    finger_indexes = None
    con_joints_indexes = None
    joint1_con = 0
    joint2_con = 0
    joint3_con = 0
    gripper_closed = False
    global objects_grasped_not_placed
    grasps_received = False

    def __init__(self, mark_pose=False):
        self.grasp_subscriber = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.grasp_callback)
        if mark_pose:
            self.mark_pose = True
            self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.p = PickPlaceInterface(group="manipulator", ee_group="endeffector", verbose=True, ns="/summit_xl/")
        self.tf = tf.TransformListener()

    def grasp_callback(self, msg):
        self.grasps = msg.grasps
        self.grasps_received = True
        if (len(msg.grasps)==0):
            pevent("No grasps found, aborting!")
        else:
            pevent("Received new grasps")


    def show_grasp_pose(self, publisher, grasp_pose):
        marker_x = Marker(
            type=Marker.ARROW,
            id=0,
            lifetime=rospy.Duration(60),
            pose=grasp_pose,
            scale=Vector3(0.04, 0.02, 0.02),
            header=Header(frame_id='summit_xl_base_footprint'),
            color=ColorRGBA(1.0, 0.0, 0.0, 0.8))
        publisher.publish(marker_x)

    def get_gpd_grasps(self):
        pevent("Waiting for grasps to arrive")
      #  while len(self.grasps) == 0:
        while (self.grasps_received == False):
            rospy.sleep(0.01)
        return self.grasps

    def generate_grasp_msgs(self, selected_grasps):
            self.grasps = []
            formatted_grasps = []
            tot_grasps = len(selected_grasps)
            cont = 0
            filtered_orientation = 0
            for i in range(0, len(selected_grasps)):
                z_axis_unit = (0, 0, -1)
                ap_axis = (selected_grasps[i].approach.x, selected_grasps[i].approach.y, selected_grasps[i].approach.z)
                angle = numpy.dot(z_axis_unit, ap_axis)
                if (angle <= 0):
                    # filter it out, because grasp coming from below the ground
                    filtered_orientation += 1
                    print(repr(filtered_orientation) + " Grasp filtered because coming from underneath the ground")
                    continue
                g = Grasp()
                g.id = "dupa"
                gp = PoseStamped()
                gp.header.frame_id = "summit_xl_base_footprint"
                org_q = self.trans_matrix_to_quaternion(selected_grasps[i])

                quat = org_q
                gp.pose.position.x = selected_grasps[i].surface.x + self.grasp_offset * selected_grasps[i].approach.x
                gp.pose.position.y = selected_grasps[i].surface.y + self.grasp_offset * selected_grasps[i].approach.y
                gp.pose.position.z = selected_grasps[i].surface.z + self.grasp_offset * selected_grasps[i].approach.z
                gp.pose.orientation.x = float(quat.elements[1])
                gp.pose.orientation.y = float(quat.elements[2])
                gp.pose.orientation.z = float(quat.elements[3])
                gp.pose.orientation.w = float(quat.elements[0])
                g.grasp_pose = gp
                g.pre_grasp_approach.direction.header.frame_id = "arm_ee_link"
                g.pre_grasp_approach.direction.vector.x = 1.0
                g.pre_grasp_approach.min_distance = 0.04
                g.pre_grasp_approach.desired_distance = 0.08
                #   g.pre_grasp_posture.joint_names = ["gripper_right_finger_joint", "gripper_left_finger_joint"]
                #   g.pre_grasp_posture.joint_names = ["arm_tool0"]
                #     g.pre_grasp_posture.header.frame_id = "arm_wrist_3_link"
                #    pos = JointTrajectoryPoint()
                #    pos.positions.append(0)
                #   pos.positions.append(0.1337)
                #   g.pre_grasp_posture.points.append(pos)

                #  g.grasp_posture.joint_names = ["gripper_right_finger_joint", "gripper_left_finger_joint"]
                #  g.grasp_posture.joint_names = ["joint_6"]
                #  pos = JointTrajectoryPoint()
                #  pos.positions.append(0.0)
                #  pos.positions.append(0.0)
                #  pos.accelerations.append(0.0)
                #  pos.accelerations.append(0.0)
                #  g.grasp_posture.points.append(pos)
                #  g.grasp_posture.header.frame_id = "hand_link"

                g.allowed_touch_objects = ["obj"]
               # g.max_contact_force = 0.0
                g.grasp_quality = selected_grasps[0].score.data
                formatted_grasps.append(g)
#                else:
#                    pass
#            print(repr(cont) + " grasps out of " + repr(tot_grasps) + " removed because of no IK_SOLUTION error")
            # sort grasps using z (get higher grasps first)
            formatted_grasps.sort(key=lambda grasp: grasp.grasp_pose.pose.position.z, reverse=True)
            return formatted_grasps

    def trans_matrix_to_quaternion(self, grasp):
        r = np.array([[grasp.approach.x, grasp.binormal.x, grasp.axis.x],
                      [grasp.approach.y, grasp.binormal.y, grasp.axis.y],
                      [grasp.approach.z, grasp.binormal.z, grasp.axis.z]])
        return Quaternion(matrix=r)

    def pick(self, grasps_list, verbose=False):
        failed_grasps = 0
        pevent("Pick sequence started")
        # Add object mesh to planning scene
        self.add_object_mesh()
        rospy.sleep(2.0)
        group.set_goal_tolerance(0.01)
        for single_grasp in grasps_list:
            if self.mark_pose:
                self.show_grasp_pose(self.marker_publisher, single_grasp.grasp_pose.pose)
                rospy.sleep(1)
            pevent("Planning grasp:")
            pprint(single_grasp.grasp_pose)
            group.set_start_state_to_current_state()
            group.detach_object("obj")

            ### start code using pick interface ###
            # pick_result = group.pick("obj", single_grasp)
            # pevent("Planner returned: " + get_moveit_error_code(pick_result))
            # if pick_result == 1:
            #    pevent("Grasp successful!")
            #    group.attach_object("obj")
            #    group.stop()
            #    group.clear_pose_targets()
            #    return single_grasp
            # else:
            #    failed_grasps += 1
            #    group.stop()
            #    group.clear_pose_targets()
            ### end code using pick interface ###

            ### start code NOT using pick interface ###
            group.set_pose_target(single_grasp.grasp_pose.pose)
            plan = group.plan()
            if (len(plan.joint_trajectory.points) != 0):
                inp = raw_input("Have a look at the planned motion. Do you want to proceed? y/n: ")[0]
                if (inp == 'y'):
                    pevent("Executing grasp: ")
                    pick_result = group.execute(plan, wait=True)
                    if pick_result == True:
                        pevent("Grasp successful!")
                        group.attach_object("obj")
                        return single_grasp
                    else:
                        failed_grasps += 1
                    group.stop()
                    group.clear_pose_targets()
                    group.clear_path_constraints()
                elif (inp == 'exit'):
                    group.stop()
                    group.clear_pose_targets()
                    group.clear_path_constraints()
                    exit(1)
        ### end code NOT using pick interface ###


       # pevent("All grasps failed. Aborting")
       # exit(1)
        self.grasps = []

    def place2(self, place_pose):
        #returns True or False if the place was successfull or not
        pevent("Place sequence started")
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = 0.516344249249
        pose_goal.position.y = -0.636391639709
        pose_goal.position.z =   0.603573918343
        pose_goal.orientation.w = 0.185884654522
        pose_goal.orientation.x = -0.681892871857
        pose_goal.orientation.y = 0.682668983936
        pose_goal.orientation.z = 0.185558646917
        group.set_start_state_to_current_state()
        group.set_pose_target(pose_goal)
        pevent("Planning pose:")
        pprint(pose_goal)
        group.set_pose_target(pose_goal)
        plan = group.plan()
        rospy.sleep(1)
        cont_plan_place=0
        place_successful=False
        while ((len(plan.joint_trajectory.points) == 0) and (cont_plan_place <10)):
            plan = group.plan()
            rospy.sleep(1)
            cont_plan_place+=1
        if (len(plan.joint_trajectory.points) != 0):
            inp = raw_input("Have a look at the planned motion. Do you want to proceed? y/n: ")[0]
            if (inp == 'y'):
                pevent("Executing place: ")
                pick_result = group.execute(plan, wait=True)
                if pick_result == True:
                    pevent("Pose successful!")
                    group.detach_object("obj")
                    place_successful=True
                else:
                    pevent("Pose failed!")
                # Calling `stop()` ensures that there is no residual movement
                group.stop()
                group.clear_pose_targets()
                place_successful = False
            elif (inp == 'exit'):
                group.stop()
                group.clear_pose_targets()
                place_successful = False
                exit(1)
        else:
            place_successful=False
            group.stop()
            group.clear_pose_targets()
        return place_successful

    def place(self, place_pose):
        pevent("Place sequence started")
        places = self.generate_place_poses(place_pose)
        place_result = self.p.place_with_retry("obj", places, support_name="<octomap>", planning_time=9001,
                                  goal_is_eef=True)
               # pevent("Planner returned: " + get_moveit_error_code(place_result.error_code.val))

    def generate_place_poses(self, initial_place_pose):
        places = list()
        l = PlaceLocation()
        l.id = "dupadupa"
        l.place_pose.header.frame_id = "summit_xl_base_footprint"
        q = Quaternion(initial_place_pose.grasp_pose.pose.orientation.w,
                        initial_place_pose.grasp_pose.pose.orientation.x,
                        initial_place_pose.grasp_pose.pose.orientation.y,
                        initial_place_pose.grasp_pose.pose.orientation.z)
# Load successful grasp pose
        l.place_pose.pose.position = initial_place_pose.grasp_pose.pose.position
        l.place_pose.pose.orientation.w = q.elements[0]
        l.place_pose.pose.orientation.x = q.elements[1]
        l.place_pose.pose.orientation.y = q.elements[2]
        l.place_pose.pose.orientation.z = q.elements[3]
# Move 20cm to the right
        l.place_pose.pose.position.y += 0.2
# Fill rest of the msg with some data
        l.post_place_posture = initial_place_pose.grasp_posture
        l.post_place_retreat = initial_place_pose.post_grasp_retreat
        l.pre_place_approach = initial_place_pose.pre_grasp_approach
        places.append(copy.deepcopy(l))
# Rotate place pose to generate more possible configurations for the planner
        m = 16  # Number of possible place poses
        for i in range(0, m - 1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * math.pi / m)
            places.append(copy.deepcopy(l))
        return places

    def add_object_mesh(self):
        obj_pose = PoseStamped()
        obj_pose.header.frame_id = "summit_xl_base_footprint"
        obj_pose.pose.position.x = 0
        obj_pose.pose.position.y = 0
        obj_pose.pose.position.z = 0
        obj_pose.pose.orientation.x = 0
        obj_pose.pose.orientation.y = 0
        obj_pose.pose.orientation.z = 0
        obj_pose.pose.orientation.w = 1
        #remove collision object from previous run
        planning.removeCollisionObject("obj")
        rospy.sleep(1)
        planning.addMesh("obj", obj_pose.pose, "object.stl")
        print("Collision object is:")
        rospy.sleep(1)
        pprint(planning.getKnownCollisionObjects())

    def set_pose_constraints(self,tol_joint1,tol_joint2,tol_joint3):
        angle_constraints = Constraints()
        joint_constraint = JointConstraint()
        angle_constraints.name = "angle"
        joint_constraint.position = self.joint1_con
        joint_constraint.tolerance_above = tol_joint1
        joint_constraint.tolerance_below = tol_joint1
        joint_constraint.weight = 0.1
        joint_constraint.joint_name = "arm_shoulder_pan_joint"
        angle_constraints.joint_constraints.append(joint_constraint)

        joint_constraint2 = JointConstraint()
        joint_constraint2.position = self.joint2_con
        joint_constraint2.tolerance_above = tol_joint2
        joint_constraint2.tolerance_below = tol_joint2
        joint_constraint2.weight = 0.2
        joint_constraint2.joint_name = "arm_wrist_2_joint"
        angle_constraints.joint_constraints.append(joint_constraint2)

        joint_constraint3 = JointConstraint()
        joint_constraint3.position = self.joint3_con
        joint_constraint3.tolerance_above = tol_joint3
        joint_constraint3.tolerance_below = tol_joint3
        joint_constraint3.weight = 0.3
        joint_constraint3.joint_name = "arm_wrist_1_joint"
        angle_constraints.joint_constraints.append(joint_constraint3)
        group.set_path_constraints(angle_constraints)


    def set_upright_constraints(self,pose):
        upright_constraints = Constraints()
        orientation_constraint = OrientationConstraint()
        upright_constraints.name = "upright"
        orientation_constraint.header = pose.header
        orientation_constraint.link_name = group.get_end_effector_link()
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 3.14
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        #orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
        orientation_constraint.weight = 1
        upright_constraints.orientation_constraints.append(orientation_constraint)
        group.set_path_constraints(upright_constraints)


    def start_con_setup(self):
        self.joints_subscriber = rospy.Subscriber('/summit_xl/joint_states', JointState, self.joints_callback)

    def stop_con_setup(self):
        self.joints_subscriber.unregister()

    def joints_callback(self, data):
        joint_names=['arm_shoulder_pan_joint','arm_wrist_1_joint','arm_wrist_2_joint']
        names = data.name
        if joint_names[0] in (names):
            j1_index = names.index("arm_shoulder_pan_joint")
            if joint_names[1] in (names):
                j2_index = names.index("arm_wrist_1_joint")
                if joint_names[2] in (names):
                    j3_index = names.index("arm_wrist_2_joint")
                    self.con_joints_indexes = (j1_index, j2_index, j3_index)
                    self.joint1_con = data.position[self.con_joints_indexes[0]]
                    self.joint2_con = data.position[self.con_joints_indexes[1]]
                    self.joint3_con = data.position[self.con_joints_indexes[2]]


    def remove_pose_constraints(self):
        group.clear_path_constraints()

    def drop_obj_on_robot(self, successful_grasp):
        pevent("Dropping object on robot")
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = -0.2
        pose_goal.position.y = 0
        pose_goal.position.z = 0.4+successful_grasp.grasp_pose.pose.position.z
        pose_goal.orientation.x = 0+successful_grasp.grasp_pose.pose.orientation.x
        pose_goal.orientation.y = 0+successful_grasp.grasp_pose.pose.orientation.y
        pose_goal.orientation.z = 0+successful_grasp.grasp_pose.pose.orientation.z
        pose_goal.orientation.w = successful_grasp.grasp_pose.pose.orientation.w
        group.set_start_state_to_current_state()
        group.set_goal_tolerance(0.05)
        group.set_pose_target(pose_goal)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        #success = group.go(wait=True)
        # if (success==True):
        #     open_gripper_summit()
        #     print("Gripper opened")
        #     group.detach_object("obj")
        # group.clear_pose_targets()
        # group.stop()
        # return success

        plan = group.plan()
        rospy.sleep(1)
        cont_plan_drop = 0
        while ((len(plan.joint_trajectory.points) == 0) and (cont_plan_drop < 10)):
            plan = group.plan()
            rospy.sleep(1)
            cont_plan_drop += 1
        if (len(plan.joint_trajectory.points) != 0):
            inp = raw_input("Have a look at the planned motion. Do you want to proceed? y/n: ")[0]
            if (inp == 'y'):
                pevent("Executing dropping: ")
                result = group.execute(plan, wait=True)
                rospy.sleep(1)
                if result == True:
                    pevent("Dropping successful!")
                    group.detach_object("obj")
                    group.stop()
                    group.clear_pose_targets()
                    open_gripper_summit()
                    print("Gripper opened")
                    return True
                else:
                    pevent("Dropping failed!")
                    group.stop()
                    group.clear_pose_targets()
                    return False
            elif (inp == 'exit'):
                group.stop()
                group.clear_pose_targets()
                group.clear_path_constraints()
                exit(1)
        else:
            pevent("Dropping position planning failed. Aborting")
            return False

    def initial_pose(self):
        pevent("Initial constrained pose sequence started")
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = 1.06285917759
        pose_goal.position.y = -0.00109632476233
        pose_goal.position.z = 0.828287780285
        pose_goal.orientation.w = 0.512402296066
        pose_goal.orientation.x = -0.512062907219
        pose_goal.orientation.y = 0.470357865095
        pose_goal.orientation.z = 0.503983080387
        group.set_start_state_to_current_state()
        group.set_goal_tolerance(0.05)
        group.set_pose_target(pose_goal)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        # group.go(joint_goal, wait=True)
        plan = group.plan()
        rospy.sleep(1)
        cont_plan = 0
        while ((len(plan.joint_trajectory.points) == 0) and (cont_plan < 10)):
            plan = group.plan()
            rospy.sleep(1)
            cont_plan += 1
        group.clear_path_constraints()
        if (len(plan.joint_trajectory.points) != 0):
            inp = raw_input("Have a look at the planned motion. Do you want to proceed? y/n: ")[0]
            if (inp == 'y'):
                pevent("Executing place: ")
                result = group.execute(plan, wait=True)
                rospy.sleep(1)
                if result == True:
                    pevent("Initial pose successful!")
                    group.stop()
                    group.clear_pose_targets()
                    return True
                else:
                    pevent("Initial pose failed!")
                    group.stop()
                    group.clear_pose_targets()
                    return False
            elif (inp == 'exit'):
                group.stop()
                group.clear_pose_targets()
                group.clear_path_constraints()
                exit(1)
        else:
            pevent("Initial position planning failed. Aborting")
            return False
       
    def wait_for_mesh_and_save(self):
      pinfo("Subscribing to pointcloud to generate mesh")
      self.obj_pc_subscriber = rospy.Subscriber("/cloud_indexed_pc_only", sensor_msgs.msg.PointCloud2 , self.obj_pointcloud_callback)

    def obj_pointcloud_callback(self, msg):
      pinfo("Pointcloud received")
      cloud = []
      for p in point_cloud2.read_points(msg, skip_nans=True):
                cloud.append([p[0], p[1], p[2]])
      create_mesh_and_save(cloud)
      pinfo("Mesh generated")
      self.obj_pc_subscriber.unregister()


    def gripper_callback(self, data):
        if (self.finger_indexes == None):
            names = data.name
            lf_index = names.index("gripper_left_finger_base_joint")
            rf_index = names.index("gripper_right_finger_base_joint")
            self.finger_indexes = (lf_index, rf_index)

        lf_joint = data.position[self.finger_indexes[0]]
        rf_joint = data.position[self.finger_indexes[1]]

        closed_range = 0.003

        if (lf_joint < closed_range and rf_joint < closed_range):
            if (not self.gripper_closed):
                pprint("Gripper closed, we probably lost the grip")
            self.gripper_closed = True

    def start_grasp_check(self):
        # subscribe to topic to derive gripper position
        self.fingers_subscriber = rospy.Subscriber('/summit_xl/joint_states', JointState, self.gripper_callback)

    def stop_grasp_check(self):
        self.fingers_subscriber.unregister()
        return self.gripper_closed


if __name__ == "__main__":
    start_time = datetime.datetime.now()
    rospy.init_node("gpd_pick_and_place",anonymous=True)
    close_gripper_summit()
    tf_listener_ = TransformListener()
    pnp = GpdPickPlace(mark_pose=True)
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name, robot_description="/summit_xl/robot_description", ns="/summit_xl")
    group.set_planner_id("BiTRRT")
  #  group.set_max_velocity_scaling_factor(0.05)
   # group.set_goal_orientation_tolerance(0.01)
    group.set_planning_time(5)
   # group.allow_replanning(True)
    planning = PlanningSceneInterface("summit_xl_base_footprint", ns="/summit_xl/")
    planning.clear()
    rospy.sleep(1)
    num_objects = 2
    succesfull_objects_placements = 0
    objects_grasped_lost = 0
    objects_grasped_not_placed = 0
    rospy.wait_for_service('/summit_xl/clear_octomap')
    clear_octomap = rospy.ServiceProxy('/summit_xl/clear_octomap', Empty)
    clear_octomap()

    for i in range (0, num_objects):
        planning.clear()
        # Subscribe for grasps
        print("--- Move Arm to Initial Position---")
      #  print "Please make sure that your robot can move freely in vertical before proceeding!"
      #  inp = raw_input("Do you want to proceed? y/n: ")[0]
      #  if (inp == 'y'):
        pnp.remove_pose_constraints()
        pnp.start_con_setup()
        rospy.sleep(1)
        pnp.set_pose_constraints(1.57, 3.14, 3.14)
        pnp.stop_con_setup()
        while (pnp.initial_pose() == False):
            print("Initial arm positioning failed!")
        print("Initial arm positioning performed")
        clear_octomap()
        #we have to add a check, so that this is called only if the initial_pose was successful
        call_pointcloud_filter_service()
        pnp.wait_for_mesh_and_save()
    # Wait for grasps from gpd, wrap them into Grasp msg format and start picking
        pnp.grasps_received = False
        selected_grasps = pnp.get_gpd_grasps()
        formatted_grasps = pnp.generate_grasp_msgs(selected_grasps)
        open_gripper_summit()
        print("Gripper opened")
        pnp.remove_pose_constraints()
        #pnp.start_con_setup()
        #rospy.sleep(1)
       # pnp.set_pose_constraints(1.57, 1.57, 1.57)
        #        pnp.stop_con_setup()
        successful_grasp = pnp.pick(formatted_grasps, verbose=True)
        if successful_grasp is not None:
            close_gripper_summit()
            print("Gripper closed")
            pnp.remove_pose_constraints()
         #   pnp.start_con_setup()
         #   rospy.sleep(1)
         #   pnp.set_pose_constraints(3.14, 1.0, 1.0)
         #   pnp.stop_con_setup()
          #  pnp.set_upright_constraints(successful_grasp.grasp_pose)
            while (pnp.drop_obj_on_robot(successful_grasp) == False):
                print("Object placing failed!")
          #  if success == False:
          #      objects_grasped_not_placed += 1
          #  else:
            succesfull_objects_placements += 1
        else:
            print("Grasp NOT performed")
            pnp.remove_pose_constraints()

        perc_successful_grasps = (100.0 * succesfull_objects_placements/num_objects)

    pnp.remove_pose_constraints()
    print(str(succesfull_objects_placements) + " out of " + str(num_objects) + " succesfull grasps, that is: %.2f" % perc_successful_grasps  + " and of the grasped once the not placed ones are " + str(objects_grasped_not_placed))
    pinfo("Demo runtime: " + str(datetime.datetime.now() - start_time))

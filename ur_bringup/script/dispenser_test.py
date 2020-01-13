#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from ur_msgs.srv import SetIO, SetIORequest, SetIOResponse
import paho.mqtt.client as mqttClient
import time
import json

dispense_reply = 'resource/dispenser/dispense/reply'
transaction_id = '00000000-0000-0000-0000-000000000000'
dispenser_id = '00000000-0000-0000-0000-000000000009'
ticket_item_id = '00000000-0000-0000-0000-000000000002'
mass = 100000
simulation_t_ms = 3000
dispense_topic = 'resource/dispenser/dispense/start'

dispense_payload = {
    'transactionId': transaction_id,
    'dispenserId': dispenser_id,
    'ticketItemId': ticket_item_id,
    'mass': mass,
    'simulation': simulation_t_ms
}

def on_connect(client, userdata, flags, rc):
  if rc == 0:
      print("Connected to broker")
      global Connected
      Connected = True                #Signal connection
  else:
      print("Connection failed")
 
def on_message(client, userdata, message):
    print("Received: " + message.payload.decode())
    global Test_finished
    Test_finished = True
    print('test_finished ' + str(test_finished))

Connected = False
Test_finished = False

broker_address= "192.168.8.100"
port = 1883
client = mqttClient.Client("Python")               #create new instance
client.on_connect=on_connect                      #attach function to callback
client.on_message=on_message
client.connect(broker_address, port=port)          #connect to broker

while True:    #Wait for connection
    client.connect(broker_address, port=port)          #connect to broker
    time.sleep(0.1)
    global Connected
    if Connected:
      break
    else:
      print("Waiting for mqtt broker")

client.subscribe(dispense_reply)
client.loop_start()

GRIPPER_PIN = 0

def call_gripper_service(state, service_client):
    try:
        req = SetIORequest()
        req.fun = SetIORequest.FUN_SET_DIGITAL_OUT
        req.pin = GRIPPER_PIN
        req.state = state
        resp = service_client(req)
        if (resp.success):
            print ("Gripper command success")
            return True
        else:
            return False
    except rospy.ServiceException, e:
        print "Service call failed: %s " % e
        return False

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator_tray"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    move_group.set_num_planning_attempts(5)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.gripper_service = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)


  def go_to_joint_state(self, shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    joint_goal = {
      'shoulder_pan_joint' : shoulder_pan,
      'shoulder_lift_joint' : shoulder_lift,
      'elbow_joint' : elbow,
      'wrist_1_joint' : wrist1,
      'wrist_2_joint' : wrist2,
      'wrist_3_joint' : wrist3,
    }

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self, pose_goal):
    assert(isinstance(pose_goal, geometry_msgs.msg.Pose))
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:


    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, waypoints):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    #

    # wpose.position.z += 0.05  # up position 
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y += 0.15 # retreat
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    while True:
      (plan, fraction) = move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
      if (fraction >= 1.0):
          break
      else:
        print("Planning failed to return a complete solution")

      if rospy.is_shutdown():
        sys.exit()

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

def main():

  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    if(call_gripper_service(0, tutorial.gripper_service)):
        print("Gripper command success")
    else:
        print("Gripper command failed")
        return

    tutorial.go_to_joint_state(-0.5062235121587358, -1.2046620944108501, 1.9430392512754255, -0.7363732523416645, -0.5064857477164486, -0.0013783038928534966)    

    while not rospy.is_shutdown():
      Test_finished = False
      
      client.publish(dispense_topic, json.dumps(dispense_payload))
      
      while True:    #Wait for dispenser
        rospy.sleep(0.1)
        print("Waiting for dispenser....")
        if Test_finished:
          break
        if rospy.is_shutdown():
          sys.exit()

      tray_base1 = geometry_msgs.msg.Pose()
      tray_base1.orientation.x = 0.707
      tray_base1.orientation.y = 0.0
      tray_base1.orientation.z = 0.00
      tray_base1.orientation.w = 0.707
      tray_base1.position.x = -0.251 
      tray_base1.position.y = -0.149 + 0.2
      tray_base1.position.z = 0.169 + 0.03

      tutorial.go_to_pose_goal(tray_base1)

      waypoints = []
      grasp = copy.deepcopy(tray_base1)
      grasp.position.y -= 0.2 # grasp position
      waypoints.append(copy.deepcopy(grasp))

      grasp.position.z -= 0.03
      waypoints.append(copy.deepcopy(grasp))
      cartesian_plan, fraction = tutorial.plan_cartesian_path(waypoints)
      print ("Sucessfully planned %f of the trajectory", fraction)
      print "============ Press `Enter` to execute a saved path ..."
      raw_input()
      tutorial.execute_plan(cartesian_plan)

      if(call_gripper_service(1, tutorial.gripper_service)):
          print("Gripper command success")
      else:
          print("Gripper command failed")
          return
      
      waypoints = []
      retreat = copy.deepcopy(grasp)
      retreat.position.z += 0.025 # grasp position
      waypoints.append(copy.deepcopy(retreat))

      retreat.position.y += 0.2 # grasp position
      waypoints.append(copy.deepcopy(retreat))

      cartesian_plan, fraction = tutorial.plan_cartesian_path(waypoints)
      print ("Sucessfully planned %f of the trajectory", fraction)
      print "============ Press `Enter` to execute a saved path ..."
      raw_input()
      tutorial.execute_plan(cartesian_plan)
      tray_base2 = geometry_msgs.msg.Pose()
      tray_base2.orientation.x = 0.707
      tray_base2.orientation.y = 0.0
      tray_base2.orientation.z = 0.00
      tray_base2.orientation.w = 0.707
      tray_base2.position.x = 0.128
      tray_base2.position.y = -0.166
      tray_base2.position.z = 0.645

      tray_int1 = geometry_msgs.msg.Pose()
      tray_int1.orientation.x = 0.707
      tray_int1.orientation.y = 0.0
      tray_int1.orientation.z = 0.00
      tray_int1.orientation.w = 0.707
      tray_int1.position.x = -0.251
      tray_int1.position.y = 0.051
      tray_int1.position.z = 0.194

      waypoints = []
      waypoints.append(copy.deepcopy(tray_int1))
      tray_int1.position.z += 0.48
      waypoints.append(copy.deepcopy(tray_int1))
      waypoints.append(copy.deepcopy(tray_base2))

      cartesian_plan, fraction = tutorial.plan_cartesian_path(waypoints)
      print ("Sucessfully planned %f of the trajectory", fraction)
      print "============ Press `Enter` to execute a saved path ..."
      raw_input()
      tutorial.execute_plan(cartesian_plan)

      joints = tutorial.move_group.get_current_joint_values()
      old_5 = joints[5]
      joints[5] += -radians(120)

      print "============ Press `Enter` to execute a movement using a joint state goal ..."
      raw_input()
      tutorial.go_to_joint_state(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5])

      rospy.sleep(3)

      tutorial.go_to_joint_state(joints[0], joints[1], joints[2], joints[3], joints[4], old_5)

      waypoints = []
      waypoints.append(copy.deepcopy(tray_int1))
      tray_int1.position.z -= 0.48
      waypoints.append(copy.deepcopy(tray_int1))
      cartesian_plan, fraction = tutorial.plan_cartesian_path(waypoints)
      tutorial.execute_plan(cartesian_plan)

      tray_base1 = geometry_msgs.msg.Pose()
      tray_base1.orientation.x = 0.707
      tray_base1.orientation.y = 0.0
      tray_base1.orientation.z = 0.00
      tray_base1.orientation.w = 0.707
      tray_base1.position.x = -0.251 
      tray_base1.position.y = -0.149 + 0.2
      tray_base1.position.z = 0.169 + 0.03

      tutorial.go_to_pose_goal(tray_base1)

      waypoints = []
      grasp = copy.deepcopy(tray_base1)
      grasp.position.y -= 0.2 # grasp position
      waypoints.append(copy.deepcopy(grasp))

      grasp.position.z -= 0.03
      waypoints.append(copy.deepcopy(grasp))
      cartesian_plan, fraction = tutorial.plan_cartesian_path(waypoints)
      print ("Sucessfully planned %f of the trajectory", fraction)
      print "============ Press `Enter` to execute a saved path ..."
      raw_input()
      tutorial.execute_plan(cartesian_plan)

      if(call_gripper_service(0, tutorial.gripper_service)):
          print("Gripper command success")
      else:
          print("Gripper command failed")
          return
      
      waypoints = []
      retreat = copy.deepcopy(grasp)
      retreat.position.z += 0.025 # grasp position
      waypoints.append(copy.deepcopy(retreat))

      retreat.position.y += 0.2 # grasp position
      waypoints.append(copy.deepcopy(retreat))

      cartesian_plan, fraction = tutorial.plan_cartesian_path(waypoints)
      print ("Sucessfully planned %f of the trajectory", fraction)
      print "============ Press `Enter` to execute a saved path ..."
      raw_input()
      tutorial.execute_plan(cartesian_plan)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
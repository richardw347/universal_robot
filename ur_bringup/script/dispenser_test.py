#!/usr/bin/env python

import sys
import copy
import rospy
from math import pi, radians
from std_msgs.msg import String
import paho.mqtt.client as mqttClient
import time
import json
from move_group_interface import MoveGroupPythonIntefaceTutorial
from gripper_service import call_gripper_service
import geometry_msgs.msg
import log_file 
import datetime

pid_reply = 'resource/dispenser/00000000-0000-0000-0000-000000000010/pid/reply'
dispense_reply = 'resource/dispenser/dispense/reply'
transaction_id = '00000000-0000-0000-0000-000000000000'
dispenser_id = '00000000-0000-0000-0000-000000000010'
ticket_item_id = '00000000-0000-0000-0000-000000000002'
mass = 50000
simulation_t_ms = 3000
pidDbg = 5
dispense_topic = 'resource/dispenser/dispense/start'

dispense_payload = {
    'transactionId': transaction_id,
    'dispenserId': dispenser_id,
    'ticketItemId': ticket_item_id,
    'mass': mass,
    'simulation': simulation_t_ms,
    'pidDbg': pidDbg
}

def clean_exit():
  if log_file.test_log_file is not None:
    log_file.test_log_file.close()
  if log_file.pid_log_file is not None:
    log_file.pid_log_file.close()
  sys.exit()
  
def on_connect(client, userdata, flags, rc):
  if rc == 0:
      print("Connected to broker")
      global Connected
      Connected = True                #Signal connection
  else:
      print("Connection failed")
 
def on_message(client, userdata, message):
    if message.topic == dispense_reply:
      global Test_finished
      Test_finished = True
      
      dispense_end =  datetime.datetime.now()
      diff = dispense_end - log_file.dispense_start
      
      dispense_reply_struct = json.loads(message.payload.decode())
      target = int(dispense_reply_struct["mass"]) / 1000.0
      actual = int(dispense_reply_struct["actualMass"]) / 1000.0
      error = target - actual
      
      log_file.target_mass = target
      log_file.actual_mass = actual
      log_file.error = error
      log_file.dispense_time = diff.seconds

      log_file.total_dispense_time  += diff.seconds
      log_file.total_target_mass += target
      log_file.total_actual_mass += actual
      log_file.total_dispense_error += error

      log_file.avg_dispense_time = log_file.total_dispense_time / log_file.test
      log_file.avg_target_mass = log_file.total_target_mass / log_file.test
      log_file.avg_actual_mass = log_file.total_actual_mass / log_file.test
      log_file.avg_dispense_error = log_file.total_dispense_error / log_file.test
      
      log_file.append_to_test_log()
      
      print('test_finished it took %d seconds to dispense %d grams, target was %d' % (diff.seconds, actual, target))

    elif message.topic == pid_reply:
      pid_reply_struct = json.loads (message.payload.decode())
      log_file.append_to_pid_log(pid_reply_struct)

Connected = False
Test_finished = False

broker_address= "192.168.8.100"
port = 1883
client = mqttClient.Client("Python")               #create new instance
client.on_connect=on_connect                      #attach function to callback
client.on_message=on_message
client.connect(broker_address, port=port)          #connect to broker
client.loop_start()

def dispense():
    global Test_finished
    Test_finished = False
    log_file.next_dispense_file()
    log_file.dispense_start = datetime.datetime.now()
    client.publish(dispense_topic, json.dumps(dispense_payload))

while True:    #Wait for connection
    client.connect(broker_address, port=port)          #connect to broker
    time.sleep(0.1)
    if Connected:
      break
    else:
      print("Waiting for mqtt broker")

client.subscribe(dispense_reply)
client.subscribe(pid_reply)

def main():

  try:
    print ("----------------------------------------------------------")
    print ("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print ("----------------------------------------------------------")
    print ("Press Ctrl-D to exit at any time")
    print ("")
    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    if(call_gripper_service(0, tutorial.gripper_service)):
        print("Gripper command success")
    else:
        print("Gripper command failed")
        return

    tutorial.go_to_joint_state(-0.5062235121587358, -1.2046620944108501, 1.9430392512754255, -0.7363732523416645, -0.5064857477164486, -0.0013783038928534966)    

    while not rospy.is_shutdown():
      global Test_finished
      Test_finished = False
      print("Set command to dispenser")
      
      dispense()
      
      while True:    #Wait for dispenser
        rospy.sleep(0.5)
        print("Waiting for dispenser....")
        if Test_finished:
          break
        if rospy.is_shutdown():
          clean_exit()

      tray_base1 = geometry_msgs.msg.Pose()
      tray_base1.orientation.x = 0.707
      tray_base1.orientation.y = 0.0
      tray_base1.orientation.z = 0.00
      tray_base1.orientation.w = 0.707
      tray_base1.position.x = -0.251 
      tray_base1.position.y = -0.149 + 0.2
      tray_base1.position.z = 0.181 + 0.03

      if (not tutorial.go_to_pose_goal(tray_base1)):
        clean_exit()

      grasp_waypoints = []
      grasp = copy.deepcopy(tray_base1)
      grasp.position.y -= 0.2 # grasp position
      grasp_waypoints.append(copy.deepcopy(grasp))

      grasp.position.z -= 0.03
      grasp_waypoints.append(copy.deepcopy(grasp))
      cartesian_plan, fraction = tutorial.plan_cartesian_path(grasp_waypoints)
      print ("Sucessfully planned %f of the trajectory", fraction)
      # print ("============ Press `Enter` to execute a saved path ...")
      # raw_input()
      if not (tutorial.execute_plan(cartesian_plan)):
        clean_exit()

      if(call_gripper_service(1, tutorial.gripper_service)):
          print("Gripper command success")
      else:
          print("Gripper command failed")
          clean_exit()
      
      retreat_waypoints = []
      retreat = copy.deepcopy(grasp)
      retreat.position.z += 0.03 # grasp position
      retreat_waypoints.append(copy.deepcopy(retreat))

      retreat.position.y += 0.2 # grasp position
      retreat_waypoints.append(copy.deepcopy(retreat))

      cartesian_plan, fraction = tutorial.plan_cartesian_path(retreat_waypoints)
      print ("Sucessfully planned %f of the trajectory", fraction)
      # print ("============ Press `Enter` to execute a saved path ...")
      # raw_input()
      
      if (not tutorial.execute_plan(cartesian_plan)):
        clean_exit()

      retreat_waypoints = []

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
      tray_int1.position.z = 0.211

      retreat_waypoints.append(copy.deepcopy(tray_int1))
      tray_int1.position.z += 0.48
      retreat_waypoints.append(copy.deepcopy(tray_int1))
      retreat_waypoints.append(copy.deepcopy(tray_base2))

      cartesian_plan, fraction = tutorial.plan_cartesian_path(retreat_waypoints)
      print ("Sucessfully planned %f of the trajectory", fraction)
      # print ("============ Press `Enter` to execute a saved path ...")
      # raw_input()
      if (not tutorial.execute_plan(cartesian_plan)):
        clean_exit()

      joints = tutorial.move_group.get_current_joint_values()
      old_5 = joints[5]
      joints[5] += -radians(120)

      tutorial.go_to_joint_state(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5])
      rospy.sleep(1)
      tutorial.go_to_joint_state(joints[0], joints[1], joints[2], joints[3], joints[4], old_5)

      tray_int2 = geometry_msgs.msg.Pose()
      tray_int2.orientation.x = 0.707
      tray_int2.orientation.y = 0.0
      tray_int2.orientation.z = 0.00
      tray_int2.orientation.w = 0.707
      tray_int2.position.x = -0.251
      tray_int2.position.y = 0.051
      tray_int2.position.z = 0.210 + 0.48

      place_waypoints = []
      place_waypoints.append(copy.deepcopy(tray_int2))
      tray_int2.position.z -= 0.48
      place_waypoints.append(copy.deepcopy(tray_int2))

      tray_base1 = geometry_msgs.msg.Pose()
      tray_base1.orientation.x = 0.707
      tray_base1.orientation.y = 0.0
      tray_base1.orientation.z = 0.00
      tray_base1.orientation.w = 0.707
      tray_base1.position.x = -0.251 
      tray_base1.position.y = -0.149 + 0.2
      tray_base1.position.z = 0.181 + 0.03

      place_waypoints.append(tray_base1)

      grasp = copy.deepcopy(tray_base1)
      grasp.position.y -= 0.2 # grasp position
      place_waypoints.append(copy.deepcopy(grasp))

      grasp.position.z -= 0.03
      place_waypoints.append(copy.deepcopy(grasp))
      cartesian_plan, fraction = tutorial.plan_cartesian_path(place_waypoints)
      print ("Sucessfully planned %f of the trajectory", fraction)
      # print "============ Press `Enter` to execute a saved path ..."
      # raw_input()
      if (not tutorial.execute_plan(cartesian_plan)):
        clean_exit()

      if(call_gripper_service(0, tutorial.gripper_service)):
          print("Gripper command success")
      else:
          print("Gripper command failed")
          return
      
      reset_waypoints = []
      retreat = copy.deepcopy(grasp)
      retreat.position.z += 0.025 # grasp position
      reset_waypoints.append(copy.deepcopy(retreat))

      retreat.position.y += 0.2 # grasp position
      reset_waypoints.append(copy.deepcopy(retreat))

      cartesian_plan, fraction = tutorial.plan_cartesian_path(reset_waypoints)
      print ("Sucessfully planned %f of the trajectory", fraction)
      # print "============ Press `Enter` to execute a saved path ..."
      # raw_input()
      if (not tutorial.execute_plan(cartesian_plan)):
        clean_exit()

  except rospy.ROSInterruptException:
    clean_exit()
    return
  except KeyboardInterrupt:
    clean_exit()
    return

if __name__ == '__main__':
  main()
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
client.loop_start()

while True:    #Wait for connection
    client.connect(broker_address, port=port)          #connect to broker
    time.sleep(0.1)
    if Connected:
      break
    else:
      print("Waiting for mqtt broker")

client.subscribe(dispense_reply)

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
      client.publish(dispense_topic, json.dumps(dispense_payload))
      
      while True:    #Wait for dispenser
        rospy.sleep(0.5)
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
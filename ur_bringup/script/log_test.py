#!/usr/bin/env python
import sys
import copy
from math import pi, radians
import paho.mqtt.client as mqttClient
import time
import json
import log_file 
import datetime

pid_reply = 'resource/dispenser/00000000-0000-0000-0000-000000000010/pid/reply'
dispense_reply = 'resource/dispenser/dispense/reply'
transaction_id = '00000000-0000-0000-0000-000000000000'
dispenser_id = '00000000-0000-0000-0000-000000000010'
ticket_item_id = '00000000-0000-0000-0000-000000000002'
mass = 50000
simulation_t_ms = 0
pidDbg = 1
dispense_topic = 'resource/dispenser/dispense/start'

dispense_payload = {
    'transactionId': transaction_id,
    'dispenserId': dispenser_id,
    'ticketItemId': ticket_item_id,
    'mass': mass,
    'simulation': simulation_t_ms,
    'pidDbg': pidDbg
}

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
      target = int(dispense_reply_struct["mass"])
      actual = int(dispense_reply_struct["actualMass"]) 
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

def terminate():
    client.disconnect()
    client.loop_stop()

def dispense():
    global Test_finished
    Test_finished = False
    log_file.next_dispense_file()
    log_file.dispense_start = datetime.datetime.now()
    client.publish(dispense_topic, json.dumps(dispense_payload))

client.subscribe(dispense_reply)

while Connected != True:    #Wait for connection
    time.sleep(0.1)
 
client.subscribe(dispense_reply)
client.subscribe(pid_reply)

try:
    dispense()
    
    while Test_finished != True:
        time.sleep(0.1)
        
    # dispense()

    # while Test_finished != True:
    #     time.sleep(0.1)

    terminate()
except KeyboardInterrupt:
    terminate()

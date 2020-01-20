#!/usr/bin/env python
import sys
import copy
import rospy
import paho.mqtt.client as mqttClient
import time
import json
import log_file
import config 
import datetime
from transport_interface.transport_interface import TransportInterface

# MQTT Topics
dispense_topic = 'resource/dispenser/dispense/start'
pid_reply = 'resource/dispenser/' + config.DISPENSER_ID + '/pid/reply'
dispense_reply = 'resource/dispenser/dispense/reply'
pid_config_topic = 'resource/dispenser/' + config.DISPENSER_ID + '/eeprom/pid'
eeprom_write_topic = 'resource/dispenser/' + config.DISPENSER_ID + '/eeprom/write'
eeprom_read_topic = 'resource/dispenser/' + config.DISPENSER_ID + '/eeprom/read'
eeprom_reply_topic = 'resource/dispenser/' + config.DISPENSER_ID + '/eeprom/reply'
snapshot_reply_topic = 'dispenser/' + config.DISPENSER_ID + '/snapshot/reply'

# Dispense command payload
dispense_payload = {
    'transactionId': '00000000-0000-0000-0000-000000000000',
    'dispenserId': config.DISPENSER_ID,
    'ticketItemId': '00000000-0000-0000-0000-000000000002',
    'mass': (config.DISPENSE_MASS * 1000),
    'simulation': 0,
    'pidDbg': 1
}

pid_config_payload = {
    "idx": 0,
    "runMx": config.PID_RUNS_MAX,
    "fKp": int(config.PID_KP * 1000),
    "fKi": int(config.PID_KI * 1000),
    "fKd": int(config.PID_KD * 1000),
    "satMx": config.PID_SAT_MIN,
    "satMn": config.PID_SAT_MAX,
    "dt": 1,
    "off": config.PID_OFFSET,
    "sampT": config.PID_SAMPLING_TIME
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
      Connected = True
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

    elif message.topic == eeprom_reply_topic:
        global eeprom_count
        eeprom_reply = json.loads(message.payload.decode())
        if "pid" in eeprom_reply:
            log_file.append_json_to_test_config(eeprom_reply)
            eeprom_count += 1
        elif "scale" in eeprom_reply:
            log_file.append_json_to_test_config(eeprom_reply)
            eeprom_count += 1
        elif "step" in eeprom_reply:
            log_file.append_json_to_test_config(eeprom_reply)
            eeprom_count += 1
        elif "dcMot" in eeprom_reply:
            log_file.append_json_to_test_config(eeprom_reply)
            eeprom_count += 1
    
    elif message.topic == snapshot_reply_topic:
        print(message.payload.decode())


Connected = False
Test_finished = False
eeprom_count = 0

client = mqttClient.Client("yoghURt")              
client.on_connect=on_connect                     
client.on_message=on_message
print("Connecting to broker address: %s:%d" %(config.MQTT_BROKER_IP, config.MQTT_BROKER_PORT))
client.connect(config.MQTT_BROKER_IP, port=config.MQTT_BROKER_PORT)     
client.loop_start()

def dispense():
    global Test_finished
    Test_finished = False
    log_file.next_dispense_file()
    log_file.dispense_start = datetime.datetime.now()
    client.publish(dispense_topic, json.dumps(dispense_payload))
    print("Sending dispense command")    

def set_pid():
    client.publish(pid_config_topic, json.dumps(pid_config_payload))

def eeprom_write():
    client.publish(eeprom_write_topic, json.dumps({}))

def eeprom_read():
    client.publish(eeprom_read_topic, json.dumps({}))

while True:    #Wait for connection
    time.sleep(0.1)
    if Connected:
      break
    else:
      print("Waiting for mqtt broker")

client.subscribe(dispense_reply)
client.subscribe(pid_reply)
client.subscribe(eeprom_reply_topic)
client.subscribe(snapshot_reply_topic)

def setup_dispenser():
    test_config = {
        "test_config":
        [{
        "food": config.FOOD_TYPE,
        "dispenser_id": config.DISPENSER_ID,
        "dispenser_type": config.DISPENSER_TYPE,
        "dispense_mass": config.DISPENSE_MASS
        }]
    }
    log_file.append_json_to_test_config(test_config)
    # print("Setting pid configuration: %s" % str(pid_config_payload))
    # set_pid()
    # time.sleep(0.5)
    # eeprom_write()
    # print("Writing configuration to eeprom")
    # time.sleep(0.5)
    eeprom_read()
    print("Reading current eeprom config")
    global eeprom_count
    while eeprom_count < 4:
        try:
            print("waiting for eeprom data")
            time.sleep(0.1)
        except KeyboardInterrupt:
            clean_exit()
            return
    log_file.test_configuration_file.close()
    

def main():

  try:
    ti = TransportInterface()
    ti.execute_initialise()

    while not rospy.is_shutdown():
        global Test_finished
        Test_finished = False
        dispense()
      
        while True:
            rospy.sleep(0.5)
            print("Waiting for dispenser....")
            if Test_finished:
                break
            if rospy.is_shutdown():
                clean_exit()

        ti.execute_tray_pick()
        ti.execute_retreat_and_dump()
  except rospy.ROSInterruptException:
    clean_exit()
    return
  except KeyboardInterrupt:
    clean_exit()
    return

if __name__ == '__main__':
  setup_dispenser()
  main()

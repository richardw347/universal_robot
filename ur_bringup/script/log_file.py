import datetime
import os 

test = 0

dispense_start = 0
dispense_time = 0

target_mass = 0
actual_mass = 0
error = 0

total_dispense_time = 0
total_target_mass = 0
total_actual_mass = 0
total_dispense_error = 0

avg_dispense_time = 0
avg_target_mass = 0
avg_actual_mass = 0
avg_dispense_error = 0

log_dir = datetime.datetime.now().strftime("%d_%m_%Y-%H%M") 
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

pid_log_file=None
test_log_file=open("./" + log_dir + "/test_log.csv", "a+")
CSV_DELIMETER = "\t"

def next_dispense_file():
    global test
    global pid_log_file
    test += 1
    test_str = str("%04d" % test)
    pid_log_file=open("./" + log_dir + "/dispenser_log_" + test_str + ".csv", "a+")

def append_to_pid_log(data):
    line = ""
    line += str(int(data["run"] + 1)) + CSV_DELIMETER
    line += str(data["cntr"]) + CSV_DELIMETER
    line += str(data["t"]) + CSV_DELIMETER
    line += str(data["sp"]) + CSV_DELIMETER
    line += str(data["cv"]) + CSV_DELIMETER
    line += str(data["fErr"] / 1000) + CSV_DELIMETER
    line += str(data["fInteg"] / 1000) + CSV_DELIMETER
    line += str(data["fDeriv"] / 1000) + CSV_DELIMETER
    line += str(data["p"]) + CSV_DELIMETER
    line += str(data["i"]) + CSV_DELIMETER
    line += str(data["d"]) + CSV_DELIMETER
    line += str(data["out"]) + "\n"
    pid_log_file.write(line)
    print("Logging: %s" %line)


def append_to_test_log():
    line = ""
    line += str(test) + CSV_DELIMETER
    line += str(datetime.datetime.now().strftime("%m/%d/%Y, %H:%M:%S")) + CSV_DELIMETER
    line += str(target_mass) + CSV_DELIMETER
    line += str(actual_mass) + CSV_DELIMETER
    line += str(error) + CSV_DELIMETER
    line += str(dispense_time) + CSV_DELIMETER
    line += str(total_target_mass) + CSV_DELIMETER
    line += str(total_actual_mass) + CSV_DELIMETER
    line += str(total_dispense_error) + CSV_DELIMETER
    line += str(total_dispense_time) + CSV_DELIMETER
    line += str(avg_target_mass) + CSV_DELIMETER
    line += str(avg_actual_mass) + CSV_DELIMETER
    line += str(avg_dispense_error) + CSV_DELIMETER
    line += str(avg_dispense_time) + "\n"
    test_log_file.write(line)
    print("Logging: %s" %line)
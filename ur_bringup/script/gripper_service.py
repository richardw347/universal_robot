import rospy
from ur_msgs.srv import SetIO, SetIORequest, SetIOResponse

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
    except rospy.ServiceException as e:
        print ("Service call failed: %s " % e)
        return True

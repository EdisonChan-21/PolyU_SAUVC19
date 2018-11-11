import asyncio
from pymavlink import mavutil
import time

class control:

    def __init__(self):
        # RC channel triming PWM
        self.rcPitchTrim = 1500
        self.rcRollTrim = 1500
        self.rcThrottleTrim = 1500
        self.rcYawTrim = 1500
        self.rcForwardTrim = 1500
        self.rcLateralTrim = 1500
        # RC input, just like joystick input
        self.rcPitch = 1500
        self.rcRoll = 1500
        self.rcThrottle = 1500
        self.rcYaw = 1500
        self.rcForward = 1500
        self.rcLateral = 1500
        # AUV current state
        self.initYaw = None
        self.initDepth = None
        self.pitch = None
        self.roll = None
        self.yaw = None
        self.depth = None # alt = depth
        self.mode = None
        self.forwardYaw = None
        # Defined parameter
        self.defaultMode = 0
        self.master = None # mavlink_connection object
        self.armed = False
        self.marginRCinput = 50
        self.gain = 0.25
        self.gainMax = 0.5 # Maximum gain
        self.updateRate = 0.5 # seconds
        self.PWMdiff = 400
        self.start = False
        self.end = False

        # self.connectArduSub()

    async def main(self):
        while (not self.end) and self.start:
            self.updateRcInput()
            self.updateInfo()
            await asyncio.sleep(self.updateRate)

    async def test(self):
        while not self.end:
            self.updateRcInput()
            await asyncio.sleep(self.updateRate)

    def readParam(self):
        # Request all parameters
        self.master.mav.param_request_list_send(
            self.master.target_system, self.master.target_component
        )
        try:
            message = self.master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
            print('name: %s\tvalue: %d' % (message['param_id'].decode("utf-8"), message['param_value']))
        except Exception as e:
            print(e)
            exit(0)

    def updateInfo(self):
        position = (self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict())
        state = (self.master.recv_match(type='AHRS2', blocking=True).to_dict())
        self.depth = int(position['alt']/10)
        self.pitch = float(("{0:.2f}".format((state['pitch']*180)/3.1415)))
        self.yaw = float(("{0:.2f}".format((state['yaw']*180)/3.1415)))
        self.roll = float(("{0:.2f}".format((state['roll']*180)/3.1415)))

    def displayInfo(self):
        print("Depth Level : " + str(self.depth) + " cm "+ " degree  |  Pitch : " + str(self.pitch) + " degree  |  Yaw : " + str(self.yaw) + " degree  |  Roll : " + str(self.roll) + " degree ")

    def connectArduSub(self):
        # Create the connection
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()
        # Start control loop
        self.start()

    def setRcChannelPwm(id, pwm=1500):
        # Set RC channel pwm value
        # Args:
        #     id (TYPE): Channel ID
        #     pwm (int, optional): Channel pwm value 1100-1900
        if id < 1:
            print("Channel does not exist.")
            return

        if (pwm>1900) or (pwm<1100):
            print("Invaild RC input PWM : ", pwm)
            return
        # We only have 8 channels
        # 1	Pitch
        # 2	Roll
        # 3	Throttle
        # 4	Yaw
        # 5	Forward
        # 6	Lateral
        # 7	Camera Pan
        # 8	Camera Tilt
        # 9	Lights 1 Level
        # 10	Lights 2 Level
        # 11	Video Switch
        if id < 9:
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[id - 1] = pwm
            master.mav.rc_channels_override_send(
                master.target_system,                # target_system
                master.target_component,             # target_component
                *rc_channel_values)                  # RC channel list, in microseconds.

    def updateRcInput(self):
        # setRcChannelPwm(1,self.rcPitch)
        # setRcChannelPwm(2,self.rcRoll)
        # setRcChannelPwm(3,self.rcThrottle)
        # setRcChannelPwm(4,self.rcYaw)
        # setRcChannelPwm(5,self.rcForward)
        # setRcChannelPwm(6,self.rcLateral)

        print(self.rcPitch,self.rcRoll,self.rcThrottle,self.rcYaw,self.rcForward,self.rcLateral)
        # Reset all to prevent forgotting reset
        self.resetRcInput()

    def resetRcInput(self):
        self.rcPitch = 1500
        self.rcRoll = 1500
        self.rcThrottle = 1500
        self.rcYaw = 1500
        self.rcForward = 1500
        self.rcLateral = 1500


    def setYaw(self,degrees, direction, relative):
        # self.master.mav.command_long_send(self.master.target_system, self.master.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,degrees,0,direction,relative,0,0,0)
        # case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        # cmd.content.yaw.angle_deg = packet.param1;      // target angle in degrees
        # cmd.content.yaw.turn_rate_dps = packet.param2;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        # cmd.content.yaw.direction = packet.param3;      // -1 = ccw, +1 = cw
        # cmd.content.yaw.relative_angle = packet.param4; // lng=0: absolute angle provided, lng=1: relative angle provided
        # break;

        pass

    def setDepth(self,depth,rate):
        pass

    def setGain(self,gain):
        if (gain>self.gainMax):
            print("Exceed Maximum gain value: ", self.gainMax)
        else:
            self.gain = gain
            print("Gain is : ", self.gain)

    def changeFlightMode(self, mode_id=0):
        # 0: 'STABILIZE',
        # 1: 'ACRO',
        # 2: 'ALT_HOLD',
        # 3: 'AUTO',
        # 4: 'GUIDED',
        # 7: 'CIRCLE',
        # 9: 'SURFACE',
        # 16: 'POSHOLD',
        # 19: 'MANUAL',
        self.master.set_mode(mode_id)
        ack = False
        while not ack:
            # Wait for ACK command
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()

            # Check if command in the same in `set_mode`
            if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
                continue

            # Print the ACK result !
            print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            break

    def arm(self):
        master.arducopter_arm()
        self.armed = True

    def disarm(self):
        master.arducopter_disarm()
        self.armed = False

    def end(self):
        self.end = True # Stop sending any rc input

    def start(self):
        self.start = True

    def pause(self):
        self.start = False

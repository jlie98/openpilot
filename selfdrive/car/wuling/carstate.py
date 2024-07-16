import copy
from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.car.wuling.values import DBC, CanBus, STEER_THRESHOLD, NetworkLocation, CAR, PREGLOBAL_CARS 

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
   
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL"]["TRANSMISSION_STATE"]

    self.lka_steering_cmd_counter = 0
    self.engineRPM = 0
    
    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.low_speed_alert = False
    self.lkas_allowed_speed = False
    self.lkas_disabled = False



  def update(self, pt_cp, cp_cam):
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons

    self.engineRPM = pt_cp.vl["ECMEngineStatus"]['EngineRPM']*0.25

    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    
    # ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]["RightSeatBelt"] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2
    
    # ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
    #             pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
    #             pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
    #             pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)
    
    ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0
    # ret.brake = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]["TRANSMISSION_STATE"], None))


    self.park_brake = pt_cp.vl["EPBStatus"]["EPBSTATUS"]
    self.pcm_acc_status = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSTATE"]
    # # dp - brake lights
    # ret.brakeLights = ret.brakePressed
    
    ret.cruiseState.enabled = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSTATE"] != 0
    # ret.cruiseActualEnabled = ret.cruiseState.enabled
    ret.cruiseState.available = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSTATE"] != 0
    ret.cruiseState.speed = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.MPH_TO_MS
    ret.steeringTorque = pt_cp.vl["PSCMSteeringAngle"]["SteeringTorque"]

    print('Cruise speed :  %s' % ret.cruiseState.speed)

    #trans state 15 "PARKING" 1 "DRIVE" 14 "BACKWARD" 13 "NORMAL"
    
    return ret

  @staticmethod
  def get_cam_can_parser(CP):
    signals = []
    checks = []
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("COUNTER", "STEERING_LKA"),
        ("ACCBUTTON", "ASCMActiveCruiseControlStatus"),
        ("ACCSTATE", "ASCMActiveCruiseControlStatus"),
        ("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"),
        ("ACCResumeAlert", "ASCMActiveCruiseControlStatus"),
        ("COUNTER_1", "ASCMActiveCruiseControlStatus"),
        ("CruiseMainOn", "AccStatus"),
        ("CruiseState", "AccStatus"),
        ("LKAS_STATE", "LkasHud"),
        ("LKA_ACTIVE", "LkasHud"),
      ]
      checks += [
        ("STEERING_LKA", 50),
        ("AccStatus", 20),
        ("ASCMActiveCruiseControlStatus", 20),
        ("LkasHud", 20),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.CAMERA)

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("TurnSignals", "BCMTurnSignals"),
      ("HighBeamsActive", "BCMTurnSignals"),
      ("SteeringWheelAngle", "PSCMSteeringAngle"),
      ("SteeringWheelRate", "PSCMSteeringAngle"),
      ("SteeringTorque", "PSCMSteeringAngle"),
      
      ("FLWheelSpd", "EBCMWheelSpdFront"),
      ("FRWheelSpd", "EBCMWheelSpdFront"),
      ("RLWheelSpd", "EBCMWheelSpdRear"),
      ("RRWheelSpd", "EBCMWheelSpdRear"),
      ("EngineRPM", "ECMEngineStatus"),
      
      ("FrontLeftDoor", "BCMDoorBeltStatus"),
      ("FrontRightDoor", "BCMDoorBeltStatus"),
      ("RearLeftDoor", "BCMDoorBeltStatus"),
      ("RearRightDoor", "BCMDoorBeltStatus"),
      ("LeftSeatBelt", "BCMDoorBeltStatus"),
      ("RightSeatBelt", "BCMDoorBeltStatus"),
      ("RIGHTSEATBEALT", "BCMDoorBelt"),
      ("LEFTSEATBEALT", "BCMDoorBelt"),
      ("EPBClosed", "EPBStatus"),
      ("Brake_Pressed", "ECMEngineStatus"),
      ("EPBSTATUS", "EPBStatus"),
      ("AVH_STATUS", "EPBStatus"),
      
      ("ACCBUTTON", "ASCMActiveCruiseControlStatus"),
      ("ACCSTATE", "ASCMActiveCruiseControlStatus"),
      ("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"),
      ("ACCResumeAlert", "ASCMActiveCruiseControlStatus"),
      ("COUNTER_1", "ASCMActiveCruiseControlStatus"),
      
      ("TRANSMISSION_STATE", "ECMPRDNL"),
      ("LKAS_STATE", "LkasHud"),
      ("LKA_ACTIVE", "LkasHud"),
      ("CruiseMainOn", "AccStatus"),
      ("CruiseState", "AccStatus"),
      ("GAS_POS", "GAS_PEDAL"),
      ("BRAKE_POS", "BRAKE_PEDAL"),
      
      ("ACC_BTN_1", "STEER_BTN"),
      ("COUNTER_1", "STEER_BTN"),
    ]

    checks = [
      ("ECMEngineStatus", 10),
      ("EPBStatus", 10),
      ("ECMPRDNL", 10),
      ("BCMDoorBeltStatus", 10),
      ("BCMDoorBelt", 10),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("PSCMSteeringAngle", 100),
      ("LkasHud", 20),
      ("AccStatus", 20),
      ("GAS_PEDAL", 10),
      ("BRAKE_PEDAL", 50),
      ("BCMTurnSignals", 30),
      ("STEER_BTN", 50),
      ("ASCMActiveCruiseControlStatus", 20),

    ]
    
     # Used to read back last counter sent to PT by camera
    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        ("COUNTER", "STEERING_LKA"),
      ]
      checks += [
        ("STEERING_LKA", 0),
      ]


    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN)

  @staticmethod
  def get_loopback_can_parser(CP):
    signals = [
      ("COUNTER", "STEERING_LKA"),
    ]

    checks = [
      ("STEERING_LKA", 0),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK, enforce_checks=False)

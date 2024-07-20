from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, CanBus, PREGLOBAL_CARS, CarControllerParams
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car import make_can_msg

from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from cereal import car

VisualAlert = car.CarControl.HUDControl.VisualAlert

class CarController():
  def __init__(self, dbc_name, CP, VM):
    # dp
    self.last_blinker_on = False
    self.blinker_end_frame = 0.
    self.enabled_last = False
    self.apply_steer_last = 0
    self.es_distance_cnt = -1
    self.es_lkas_cnt = -1
    self.cruise_button_prev = 0
    self.steer_rate_limited = False
    self.steer_alert_last = False
    self.lkas_action = 0
    
    self.apply_gas = 0
    self.apply_brake = 0

    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)

    print(CP.carFingerprint)
    print(DBC[CP.carFingerprint])
    
    self.p = CarControllerParams()
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed, left_line, right_line, left_lane_depart, right_lane_depart):

    P = self.p

    can_sends = []
    steer_alert = visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lkas_active = True
    
    if CC.latActive:
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)
    else:
      apply_angle = CS.out.steeringAngleDeg

    self.apply_angle_last = apply_angle

    # print('car controller: steeringAngleDeg:', actuators.steeringAngleDeg)
    # print('car controller: apply_angle_last:',  self.apply_angle_last)
    # print('car controller: vEgo:',  CS.out.vEgo)
    # print('car controller: apply_angle:',  apply_angle)

    can_sends.append(wulingcan.create_steering_control(
      self.packer_pt, apply_angle, self.frame, CC.enabled))


    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
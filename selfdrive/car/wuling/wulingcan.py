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

    self.p = CarControllerParams()
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed, left_line, right_line, left_lane_depart, right_lane_depart):

    can_sends = []

    if (frame % self.p.STEER_STEP) == 0:
      apply_steer = int(round(actuators.steer * self.p.STEER_MAX))

      new_steer = int(round(apply_steer))
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
      self.steer_rate_limited = new_steer != apply_steer

      if not c.active:
        apply_steer = 0   

      can_sends.append(wulingcan.create_steering_control(self.packer, apply_steer, frame, 1))
      self.apply_steer_last = apply_steer

    
    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.p.STEER_MAX


    return new_actuators, can_sends
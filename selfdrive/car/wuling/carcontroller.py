from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, CanBus, PREGLOBAL_CARS, CarControllerParams
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car import make_can_msg

from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from cereal import car
from common.numpy_fast import clip

VisualAlert = car.CarControl.HUDControl.VisualAlert

def apply_wuling_steer_angle_limits(apply_angle, actual_angle, v_ego):
  # pick angle rate limits based on wind up/down
  ANGLE_RATE_LIMIT_UP = 3       # maximum allow 150 degree per second, 100Hz loop means 1.5
  ANGLE_RATE_LIMIT_DOWN = 3
  
  steer_up = actual_angle * apply_angle >= 0. and abs(apply_angle) > abs(actual_angle)
  rate_limits = ANGLE_RATE_LIMIT_UP if steer_up else ANGLE_RATE_LIMIT_DOWN

  return clip(apply_angle, actual_angle - rate_limits, actual_angle + rate_limits)

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    
    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False
    
    self.params = CarControllerParams()
    
    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])


  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed, left_line, right_line, left_lane_depart, right_lane_depart):

    P = self.params
    
    # Send CAN commands.
    can_sends = []
    
    apply_angle = apply_wuling_steer_angle_limits(actuators.steeringAngleDeg, CS.out.steeringAngleDeg, CS.out.vEgo)

    # Steering (50Hz)
    # Avoid GM EPS faults when transmitting messages too close together: skip this transmit if we just received the
    # next Panda loopback confirmation in the current CS frame.
    if (frame % 2) == 0:
      lkas_enabled = True
      # if !lkas_enabled:
      # else:
      #   apply_angle = CS.out.steeringAngleDeg
        
      idx = (CS.lka_steering_cmd_counter + 1) % 4
    
      can_sends.append(wulingcan.create_steering_control(self.packer_pt, apply_angle, idx, lkas_enabled))
    
    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle
    
    print('Steer :  %s' % apply_angle)

    return new_actuators, can_sends
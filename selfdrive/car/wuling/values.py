from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu
NetworkLocation = car.CarParams.NetworkLocation

class CarControllerParams:
  STEER_MAX = 300  # Safety limit, not LKA max. Trucks use 600.
  STEER_STEP = 2  # control frames per command
  STEER_MAX = 261         # 262 faults
  STEER_DELTA_UP = 3      # 3 is stock. 100 is fine. 200 is too much it seems
  STEER_DELTA_DOWN = 3    # no faults on the way down it seems
  STEER_ERROR_MAX = 80
  ADAS_KEEPALIVE_STEP = 100
  MIN_STEER_SPEED = 3.  # m/s

  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 3    # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1        # from dbc

  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[5., .8, .15])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[5., 3.5, 0.4])
  
class CANBUS:
  pt = 0
  cam = 2

class CAR:
  ALVEZ = "WULING ALVEZ"
class CruiseButtons:
  INIT = 0
  UNPRESS = 1
  RES_ACCEL = 2
  DECEL_SET = 3
  MAIN = 5
  CANCEL = 6

class AccState:
  OFF = 0
  ACTIVE = 1
  FAULTED = 3
class CanBus:
  POWERTRAIN = 0
  OBSTACLE = 1
  CAMERA = 2
  CHASSIS = 2
  SW_GMLAN = 3
  LOOPBACK = 128
  DROPPED = 192
  
FINGERPRINTS = {
  CAR.ALVEZ: [{
    193: 8, 197: 8, 201: 8, 225: 8, 258: 8, 288: 5, 296: 8, 298: 8, 320: 4, 381: 8, 401: 8, 411: 8, 413: 8, 451: 8, 454: 8, 481: 8, 485: 8, 489: 8, 497: 8, 501: 8, 549: 8, 560: 8, 563: 8, 608: 8, 611: 8, 617: 8, 840: 6, 842: 6, 844: 6, 846: 6, 880: 8, 883: 8, 996: 8, 997: 8, 1041: 8, 1043: 8, 1045: 8, 1047: 8, 1053: 8, 1065: 8, 1217: 8, 1225: 8, 1341: 8, 1381: 8, 1406: 8, 1417: 8, 1538: 8, 1541: 8, 1552: 8, 1569: 8
  }]
}

FW_VERSIONS = {
   CAR.ALVEZ: {}
}
DBC = {
  CAR.ALVEZ: dbc_dict('wuling_almazrs_generated', None),
}

STEER_THRESHOLD = 0

PREGLOBAL_CARS = []
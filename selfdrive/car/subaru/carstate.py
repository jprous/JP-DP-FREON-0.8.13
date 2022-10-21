import copy
from cereal import car
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.car.subaru.values import DBC, STEER_THRESHOLD, CAR, PREGLOBAL_CARS
#import cereal.messaging as messaging
#from selfdrive.controls.lib.longitudinal_planner import Planner

#CruiseState = car.CarState.CruiseState

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["Transmission"]["Gear"]
    #self.longitudinal_planner = Planner(CP)
    self.hudspd = 0;
    self.steerout = 0;
    self.speed_limit = 255;
    #self.sm = messaging.SubMaster(['liveMapData'], poll=['liveMapData'], ignore_avg_freq=['radarState'])

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()


    #self.sm.update(0)
    #if self.sm.updated['liveMapData']:
    #self.speed_limit = self.sm['liveMapData'].speedLimit
    
    #ret.cruiseState.speedLimit = self.longitudinal_planner.speed_limit_controller.speed_limit
    
    #self.sm = messaging.SubMaster(['liveMapData'])
    #sm = messaging.SubMaster(['liveMapData'], ignore_avg_freq=['radarState'])
    #self.sm.update(0)
    #if sm['LiveMapData'].speedLimitValid:
    #  ret.cruiseState.speedLimit = sm['liveMapData'].speedLimit
    #else:
    #  ret.cruiseState.speedLimit = 250
      
    #ret.cruiseState.speedLimit = 250  
    #ret.cruiseState.speedLimit = int(sm['liveMapData'].lastGpsBearingAccuracyDeg)
    ##ret.cruiseState.speedLimit = 250
    ##self.sm.update(0)
    ##ret.cruiseState.speedLimit = int(self.sm['liveMapData'].lastGpsTimestamp/1000000000)
    
    #ret.cruiseState.speedLimit = messaging.SubMaster(['carState']).cruiseState.speedLimit
    
    #sm = messaging.SubMaster
    #self.speedLimit = sm['longitudinalPlan'].speedLimit
    #ret.cruiseState.speedLimit = round(speedLimit * CV.MS_TO_KPH)
    
    #ret.cruiseState.speedLimit = CruiseState.speedLimit
    
    ret.gas = cp.vl["Throttle"]["Throttle_Pedal"] / 255.
    ret.gasPressed = ret.gas > 1e-5
    if self.car_fingerprint in PREGLOBAL_CARS:
      ret.brakePressed = cp.vl["Brake_Pedal"]["Brake_Pedal"] > 2
    else:
      ret.brakePressed = cp.vl["Brake_Status"]["Brake"] == 1

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["Wheel_Speeds"]["FL"],
      cp.vl["Wheel_Speeds"]["FR"],
      cp.vl["Wheel_Speeds"]["RL"],
      cp.vl["Wheel_Speeds"]["RR"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    # Kalman filter, even though Subaru raw wheel speed is heaviliy filtered by default
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    # continuous blinker signals for assisted lane change
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["Dashlights"]["LEFT_BLINKER"], cp.vl["Dashlights"]["RIGHT_BLINKER"])

    if self.CP.enableBsm:
      ret.leftBlindspot = (cp.vl["BSD_RCTA"]["L_ADJACENT"] == 1) or (cp.vl["BSD_RCTA"]["L_APPROACHING"] == 1)
      ret.rightBlindspot = (cp.vl["BSD_RCTA"]["R_ADJACENT"] == 1) or (cp.vl["BSD_RCTA"]["R_APPROACHING"] == 1)

    can_gear = int(cp.vl["Transmission"]["Gear"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.steeringAngleDeg = cp.vl["Steering_Torque"]["Steering_Angle"]
    ret.steeringTorque = cp.vl["Steering_Torque"]["Steer_Torque_Sensor"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD[self.car_fingerprint]

    ret.cruiseState.enabled = cp.vl["CruiseControl"]["Cruise_Activated"] != 0
    ret.cruiseState.available = cp.vl["CruiseControl"]["Cruise_On"] != 0
    ret.cruiseState.speed = cp_cam.vl["ES_DashStatus"]["Cruise_Set_Speed"] * CV.KPH_TO_MS

    if (self.car_fingerprint in PREGLOBAL_CARS and cp.vl["Dash_State2"]["UNITS"] == 1) or \
       (self.car_fingerprint not in PREGLOBAL_CARS and cp.vl["Dashlights"]["UNITS"] == 1):
      ret.cruiseState.speed *= CV.MPH_TO_KPH

    ret.seatbeltUnlatched = cp.vl["Dashlights"]["SEATBELT_FL"] == 1
    ret.doorOpen = any([cp.vl["BodyInfo"]["DOOR_OPEN_RR"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_RL"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_FR"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_FL"]])
    ret.steerError = cp.vl["Steering_Torque"]["Steer_Error_1"] == 1

    if self.car_fingerprint in PREGLOBAL_CARS:
      self.cruise_button = cp_cam.vl["ES_Distance"]["Cruise_Button"]
      self.ready = not cp_cam.vl["ES_DashStatus"]["Not_Ready_Startup"]
    else:
      ret.steerWarning = cp.vl["Steering_Torque"]["Steer_Warning"] == 1
      ret.cruiseState.nonAdaptive = cp_cam.vl["ES_DashStatus"]["Conventional_Cruise"] == 1
      self.es_lkas_msg = copy.copy(cp_cam.vl["ES_LKAS_State"])
      self.cruise_state = cp_cam.vl["ES_DashStatus"]["Cruise_State"]
    self.car_follow = cp_cam.vl["ES_Distance"]["Car_Follow"]
    self.close_distance = cp_cam.vl["ES_Distance"]["Close_Distance"]
    self.throttle_msg = copy.copy(cp.vl["Throttle"])
    self.es_distance_msg = copy.copy(cp_cam.vl["ES_Distance"])
    self.es_dashstatus_msg = copy.copy(cp_cam.vl["ES_DashStatus"])
    self.es_lanecenter_msg = copy.copy(cp_cam.vl["ES_LANE_CENTER"])
    self.es_status_2_msg = copy.copy(cp_cam.vl["ES_Status_2"])
    self.es_new_tst_2_msg = copy.copy(cp_cam.vl["NEW_TST_2"])
    self.es_lkas_master_msg = copy.copy(cp_cam.vl["ES_LKAS_Master"])
    self.es_unknown1_msg = copy.copy(cp_cam.vl["ES_UNKNOWN1"])
    #self.prev_cruise_buttons = self.cruise_buttons
    self.sw_cruise_buttons_msg = copy.copy(cp.vl["Cruise_Buttons"]) 
    self.sw_ss_state_msg = copy.copy(cp.vl["START_STOP_STATE"]) 
    self.steering_torque_msg = copy.copy(cp.vl["Steering_Torque"])
    #self.dashlights_msg = copy.copy(cp.vl["Dashlights"])
    #self.es_lkas_steer_msg = copy.copy(cp_cam.vl["ES_LKAS"])
    self.es_steerjp_msg = copy.copy(cp.vl["ES_STEER_JP"])
    
    self.cruise_cancel_btn = 0
    if ((cp_cam.vl["ES_Distance"]["Cruise_Set"] == 1) and (cp_cam.vl["ES_Distance"]["Cruise_Resume"] == 1)):
      self.cruise_cancel_btn = 1

    # dp - brake lights
    ret.brakeLights = ret.brakePressed
    #ret.engineRPM = cp.vl["Throttle"]['Engine_RPM']
    #ret.engineRPM = abs(cp.vl["Steering_Torque"]["Steer_Torque_Sensor"])  #JP test
    #ret.engineRPM = cp.vl["Cruise_Buttons"]["Resume"]
    #ret.engineRPM = cp_cam.vl["ES_Distance"]["Cruise_Resume"]
    #ret.engineRPM = self.hudspd
    ret.engineRPM = self.steerout
    #ret.engineRPM = self.cruise_cancel_btn
    #ret.engineRPM = self.speed_limit
    # if (self.speed_limit != 0):
      # ret.engineRPM = self.speed_limit
    # else:
      # ret.engineRPM = self.steerout
      
    #BUS signals to checks
    #HEX 33a
    #HEX 34a
    #HEX 124 on Wilderness has steering: bit 40 size 11
      
    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("Counter", "Steering_Torque"),
      ("Steer_Torque_Sensor", "Steering_Torque"),
      ("Steering_Angle", "Steering_Torque"),
      ("Steer_Error_1", "Steering_Torque"),
      ("Signal1", "Steering_Torque"),
      ("Cruise_On", "CruiseControl"),
      ("Cruise_Activated", "CruiseControl"),
      ("Brake_Pedal", "Brake_Pedal"),
      ("Throttle_Pedal", "Throttle"),
      ("Counter", "Dashlights"),
      ("LEFT_BLINKER", "Dashlights"),
      ("RIGHT_BLINKER", "Dashlights"),
      ("SEATBELT_FL", "Dashlights"),
      ("STOP_START", "Dashlights"),
      ("FL", "Wheel_Speeds"),
      ("FR", "Wheel_Speeds"),
      ("RL", "Wheel_Speeds"),
      ("RR", "Wheel_Speeds"),
      ("DOOR_OPEN_FR", "BodyInfo"),
      ("DOOR_OPEN_FL", "BodyInfo"),
      ("DOOR_OPEN_RR", "BodyInfo"),
      ("DOOR_OPEN_RL", "BodyInfo"),
      ("Gear", "Transmission"),
      
      ("Counter","Cruise_Buttons"),
      ("Signal1","Cruise_Buttons"),
      ("Main","Cruise_Buttons"),
      ("Set","Cruise_Buttons"),
      ("Resume","Cruise_Buttons"),
      ("Signal2","Cruise_Buttons"),
      
      ("Counter","ES_STEER_JP"),
      ("BLANK_SIGNAL","ES_STEER_JP"),
      ("NEW_SIGNAL_1","ES_STEER_JP"),
      ("STEER_ANGLE","ES_STEER_JP"),
      ("STEER_OUTPUT","ES_STEER_JP"),
      
      ("Counter","START_STOP_STATE"),
      ("Signal1","START_STOP_STATE"),
      
      
    ]

    checks = [
      # sig_address, frequency
      ("Throttle", 100),
      ("Dashlights", 10),
      ("Brake_Pedal", 50),
      ("Wheel_Speeds", 50),
      ("Transmission", 100),
      ("Steering_Torque", 50),
      ("BodyInfo", 1),
      ("Cruise_Buttons", 20),
      ("ES_STEER_JP", 100),
      ("START_STOP_STATE", 20),
    ]

    if CP.enableBsm:
      signals += [
        ("L_ADJACENT", "BSD_RCTA"),
        ("R_ADJACENT", "BSD_RCTA"),
        ("L_APPROACHING", "BSD_RCTA"),
        ("R_APPROACHING", "BSD_RCTA"),
      ]
      checks.append(("BSD_RCTA", 17))

    if CP.carFingerprint not in PREGLOBAL_CARS:
      signals += [
        ("Counter", "Throttle"),
        ("Signal1", "Throttle"),
        ("Engine_RPM", "Throttle"),
        ("Signal2", "Throttle"),
        ("Throttle_Pedal", "Throttle"),
        ("Throttle_Cruise", "Throttle"),
        ("Throttle_Combo", "Throttle"),
        ("Signal1", "Throttle"),
        ("Off_Accel", "Throttle"),
        ("Steer_Warning", "Steering_Torque"),
        ("Brake", "Brake_Status"),
        ("UNITS", "Dashlights"),
      ]

      checks += [
        ("Dashlights", 10),
        ("BodyInfo", 10),
        ("Brake_Status", 50),
        ("CruiseControl", 20),
        
      ]
    else:
      signals += [
        ("Throttle_Pedal", "Throttle"),
        ("Counter", "Throttle"),
        ("Signal1", "Throttle"),
        ("Not_Full_Throttle", "Throttle"),
        ("Signal2", "Throttle"),
        ("Engine_RPM", "Throttle"),
        ("Off_Throttle", "Throttle"),
        ("Signal3", "Throttle"),
        ("Throttle_Cruise", "Throttle"),
        ("Throttle_Combo", "Throttle"),
        ("Throttle_Body", "Throttle"),
        ("Off_Throttle_2", "Throttle"),
        ("Signal4", "Throttle"),
        ("UNITS", "Dash_State2"),
        ("Brake_Pedal", "Brake_Pedal"),
        
      ]

      checks.append(("BodyInfo", 1))
      checks.append(("Dash_State2", 1))
      checks.append(("Brake_Pedal", 50))
      checks.append(("CruiseControl", 50))

    if CP.carFingerprint == CAR.FORESTER_PREGLOBAL:
      checks += [
        ("Dashlights", 20),
        ("BodyInfo", 1),
        ("CruiseControl", 50),
      ]

    if CP.carFingerprint in (CAR.LEGACY_PREGLOBAL, CAR.OUTBACK_PREGLOBAL, CAR.OUTBACK_PREGLOBAL_2018):
      checks += [
        ("Dashlights", 10),
        ("CruiseControl", 50),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    if CP.carFingerprint in PREGLOBAL_CARS:
      signals = [
        ("Cruise_Set_Speed", "ES_DashStatus"),
        ("Not_Ready_Startup", "ES_DashStatus"),

        ("Cruise_Throttle", "ES_Distance"),
        ("Signal1", "ES_Distance"),
        ("Car_Follow", "ES_Distance"),
        ("Signal2", "ES_Distance"),
        ("Brake_On", "ES_Distance"),
        ("Distance_Swap", "ES_Distance"),
        ("Standstill", "ES_Distance"),
        ("Signal3", "ES_Distance"),
        ("Close_Distance", "ES_Distance"),
        ("Signal4", "ES_Distance"),
        ("Standstill_2", "ES_Distance"),
        ("Cruise_Fault", "ES_Distance"),
        ("Signal5", "ES_Distance"),
        ("Counter", "ES_Distance"),
        ("Signal6", "ES_Distance"),
        ("Cruise_Button", "ES_Distance"),
        ("Signal7", "ES_Distance"),
      ]

      checks = [
        ("ES_DashStatus", 20),
        ("ES_Distance", 20),
      ]
    else:
      signals = [      
                
        ("Counter", "ES_DashStatus"),
        ("PCB_Off", "ES_DashStatus"),
        ("LDW_Off", "ES_DashStatus"),
        ("Signal1", "ES_DashStatus"),
        ("Cruise_State_Msg", "ES_DashStatus"),
        ("LKAS_State_Msg", "ES_DashStatus"),
        ("Signal2", "ES_DashStatus"),
        ("Cruise_Soft_Disable", "ES_DashStatus"),
        ("EyeSight_Status_Msg", "ES_DashStatus"),
        ("Signal3", "ES_DashStatus"),
        ("Cruise_Distance", "ES_DashStatus"),
        ("Signal4", "ES_DashStatus"),
        ("Conventional_Cruise", "ES_DashStatus"),
        ("Signal5", "ES_DashStatus"),
        ("Cruise_Disengaged", "ES_DashStatus"),
        ("Cruise_Activated", "ES_DashStatus"),
        ("Signal6", "ES_DashStatus"),
        ("Cruise_Set_Speed", "ES_DashStatus"),
        ("Cruise_Fault", "ES_DashStatus"),
        ("Cruise_On", "ES_DashStatus"),
        ("Display_Own_Car", "ES_DashStatus"),
        ("Brake_Lights", "ES_DashStatus"),
        ("Car_Follow", "ES_DashStatus"),
        ("Signal7", "ES_DashStatus"),
        ("Far_Distance", "ES_DashStatus"),
        ("Cruise_State", "ES_DashStatus"),

        ("Counter", "ES_Distance"),
        ("Signal1", "ES_Distance"),
        ("Cruise_Fault", "ES_Distance"),
        ("Cruise_Throttle", "ES_Distance"),
        ("Signal2", "ES_Distance"),
        ("Car_Follow", "ES_Distance"),
        ("Signal3", "ES_Distance"),
        ("Cruise_Brake_Active", "ES_Distance"),
        ("Distance_Swap", "ES_Distance"),
        ("Cruise_EPB", "ES_Distance"),
        ("Signal4", "ES_Distance"),
        ("Close_Distance", "ES_Distance"),
        ("Signal5", "ES_Distance"),
        ("Cruise_Cancel", "ES_Distance"),
        ("Cruise_Set", "ES_Distance"),
        ("Cruise_Resume", "ES_Distance"),
        ("Signal6", "ES_Distance"),

        ("Counter", "ES_LKAS_State"),
        ("LKAS_Alert_Msg", "ES_LKAS_State"),
        ("Signal1", "ES_LKAS_State"),
        ("LKAS_ACTIVE", "ES_LKAS_State"),
        ("LKAS_Dash_State", "ES_LKAS_State"),
        ("Signal2", "ES_LKAS_State"),
        ("Backward_Speed_Limit_Menu", "ES_LKAS_State"),
        ("LKAS_Left_Line_Enable", "ES_LKAS_State"),
        ("LKAS_Left_Line_Light_Blink", "ES_LKAS_State"),
        ("LKAS_Right_Line_Enable", "ES_LKAS_State"),
        ("LKAS_Right_Line_Light_Blink", "ES_LKAS_State"),
        ("LKAS_Left_Line_Visible", "ES_LKAS_State"),
        ("LKAS_Right_Line_Visible", "ES_LKAS_State"),
        ("LKAS_Alert", "ES_LKAS_State"),
        ("Signal3", "ES_LKAS_State"),
        ("LKAS_Enable_1", "ES_LKAS_State"),
        ("NEW_SIGNAL_1", "ES_LKAS_State"),
        ("LANE_CENTRE_SET", "ES_LKAS_State"),
        ("LEFT_RIGHT_SET", "ES_LKAS_State"),
        ("NEW_SIGNAL_2", "ES_LKAS_State"),
        ("LKAS_SET", "ES_LKAS_State"),
        ("ACC_ACEL_CHAR", "ES_LKAS_State"),
        ("Signal5", "ES_LKAS_State"),

        ("Counter", "ES_LANE_CENTER"),
        ("Signal1", "ES_LANE_CENTER"),
        ("Signal2", "ES_LANE_CENTER"),
        ("Signal3", "ES_LANE_CENTER"),
        
        ("Counter", "ES_Status_2"),
        ("Signal1", "ES_Status_2"),
        ("Signal2", "ES_Status_2"),
        ("Signal3", "ES_Status_2"),
        
        ("Counter", "NEW_TST_2"),
        ("NEW_SIGNAL_1", "NEW_TST_2"),
        
        ("Counter", "ES_LKAS_Master"),
        ("Signal1", "ES_LKAS_Master"),
        
        ("Counter", "ES_UNKNOWN1"),
        ("Signal1", "ES_UNKNOWN1"),
        
        #("Counter","ES_LKAS"),
        #("SET_1","ES_LKAS"),
        #("SET_2","ES_LKAS"),
        #("LKAS_Request","ES_LKAS"),
        #("LKAS_Output","ES_LKAS"),        
      ]

      checks = [
        ("ES_DashStatus", 10),
        ("ES_Distance", 20),
        ("ES_LKAS_State", 10),
        ("ES_LANE_CENTER", 10),
        ("ES_Status_2", 10),
        ("NEW_TST_2", 50),
        ("ES_LKAS_Master", 20),
        ("ES_UNKNOWN1", 20),
        #("ES_LKAS", 20),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)

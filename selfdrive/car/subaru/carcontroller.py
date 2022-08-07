from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.subaru import subarucan
from selfdrive.config import Conversions as CV
from selfdrive.car.subaru.values import DBC, PREGLOBAL_CARS, CarControllerParams
from opendbc.can.packer import CANPacker
from common.dp_common import common_controller_ctrl

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    # dp
    self.last_blinker_on = False
    self.blinker_end_frame = 0.
    
    self.apply_steer_last = 0
    self.es_distance_cnt = -1
    self.es_lkas_cnt = -1
    self.es_status_2_cnt = -1
    self.cruise_buttons_cnt = -1
    self.es_dashstatus_cnt = -1
    self.throttle_cnt = -1
    self.brake_pedal_cnt = -1                                                 
    self.cruise_button_prev = 0
    self.prev_close_distance = 0
    self.prev_standstill = False
    self.standstill_start = 0                                                       
    self.steer_rate_limited = False
    self.frame = 0
    self.sng_acc_resume = False
    self.sng_acc_resume_cnt = -1
    self.manual_hold = False
    self.prev_cruise_state = 0
    self.prev_close_distance = 0
    
    self.speed_check_cnt = -1
    
    self.dn_button_press = False
    self.dn_button_press_cnt = -1
    
    self.up_button_press = False
    self.up_button_press_cnt = -1

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, left_line, right_line, left_lane_depart, right_lane_depart, dragonconf):

    can_sends = []

    # *** steering ***
    if (frame % self.p.STEER_STEP) == 0:

      apply_steer = int(round(actuators.steer * self.p.STEER_MAX))

      # limits due to driver torque

      new_steer = int(round(apply_steer))
      apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
      self.steer_rate_limited = new_steer != apply_steer

      if not c.active:
        apply_steer = 0

      # dp
      blinker_on = CS.out.leftBlinker or CS.out.rightBlinker
      if not enabled:
        self.blinker_end_frame = 0
      if self.last_blinker_on and not blinker_on:
        self.blinker_end_frame = frame + dragonconf.dpSignalOffDelay
      apply_steer = common_controller_ctrl(enabled,
                                           dragonconf,
                                           blinker_on or frame < self.blinker_end_frame,
                                           apply_steer, CS.out.vEgo)
      self.last_blinker_on = blinker_on

      if CS.CP.carFingerprint in PREGLOBAL_CARS:
        can_sends.append(subarucan.create_preglobal_steering_control(self.packer, apply_steer, frame, self.p.STEER_STEP))
      else:
        can_sends.append(subarucan.create_steering_control(self.packer, apply_steer, frame, self.p.STEER_STEP))

      self.apply_steer_last = apply_steer

    # *** speed control - check once per second ***
    cspeed_dn_cmd = False
    cspeed_up_cmd = False
    
    new_cspeed = 0    #set the desired speed here to test
    
    if ((CS.cruise_state == 0) and (c.enabled) and (new_cspeed !=0) and (CS.out.cruiseState.enabled > 0)):
      if (self.speed_check_cnt > 200):
      
        #Check down first - but to a miminum of 30 kph
        if ((new_cspeed > 29) and (new_cspeed < CS.out.cruiseState.speed * CV.MS_TO_KPH)):
          self.dn_button_press = True  #press the down button
        
        else:
          #Check up more than 20 kph, but to a max of 131 kph
          if ((new_cspeed < 131) and (new_cspeed > (CS.out.cruiseState.speed * CV.MS_TO_KPH))):
            self.up_button_press = True  #press the down button
        
        self.speed_check_cnt = -1
      else:
        self.speed_check_cnt += 1

        #press the down button if required for 5 frames
      if self.dn_button_press:
        if ((self.dn_button_press_cnt < 5) and (self.up_button_press == False)):
          cspeed_dn_cmd = True
          self.dn_button_press_cnt += 1
        else:
          self.dn_button_press = False
          self.dn_button_press_cnt = -1
         
         #press the up button if required for 5 frames
      if self.up_button_press:
        if ((self.up_button_press_cnt < 5) and (self.dn_button_press == False)):
          cspeed_up_cmd = True
          self.up_button_press_cnt += 1
        else:
          self.up_button_press = False
          self.up_button_press_cnt = -1
    
    
    # *** stop and go ***
    throttle_cmd = False
    
   
    #if CS.CP.carFingerprint not in PREGLOBAL_CARS:
    # Record manual hold set while in standstill and no car in front
    if CS.out.standstill and self.prev_cruise_state == 1 and CS.cruise_state == 3 and CS.car_follow == 0:
      self.manual_hold = True
    # Cancel manual hold when car starts moving
    if not CS.out.standstill:
      self.manual_hold = False
    if (c.enabled                                            # ACC active
        and not self.manual_hold
        and CS.car_follow == 1                             # lead car
        and CS.cruise_state == 3                           # ACC HOLD (only with EPB)
        and CS.out.standstill                              # must be standing still
        and CS.close_distance > self.p.ACC_MIN_DIST        # acc resume min trigger threshold (m)
        and CS.close_distance < self.p.ACC_MAX_DIST        # acc resume max trigger threshold (m)
        and CS.close_distance > self.prev_close_distance): # distance with lead car is increasing
      self.sng_acc_resume = True
    self.prev_cruise_state = CS.cruise_state

    if self.sng_acc_resume:
      if self.sng_acc_resume_cnt < 5:
        throttle_cmd = True
        self.sng_acc_resume_cnt += 1
      else:
        self.sng_acc_resume = False
        self.sng_acc_resume_cnt = -1

    # Cancel ACC if stopped, brake pressed and not stopped behind another car
    if c.enabled and CS.out.brakePressed and CS.car_follow == 0 and CS.out.standstill:
      pcm_cancel_cmd = True

    self.prev_close_distance = CS.close_distance

    
    # *** alerts and pcm cancel ***

    if CS.CP.carFingerprint in PREGLOBAL_CARS:
      if self.es_distance_cnt != CS.es_distance_msg["Counter"]:
        # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
        # disengage ACC when OP is disengaged
        if pcm_cancel_cmd:
          cruise_button = 1
        # turn main on if off and past start-up state
        elif not CS.out.cruiseState.available and CS.ready:
          cruise_button = 1
        else:
          cruise_button = CS.cruise_button

        # unstick previous mocked button press
        if cruise_button == 1 and self.cruise_button_prev == 1:
          cruise_button = 0
        self.cruise_button_prev = cruise_button

        can_sends.append(subarucan.create_preglobal_es_distance(self.packer, cruise_button, CS.es_distance_msg))
        self.es_distance_cnt = CS.es_distance_msg["Counter"]

    else:
      if self.es_distance_cnt != CS.es_distance_msg["Counter"]:
        can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg, pcm_cancel_cmd, cspeed_dn_cmd, cspeed_up_cmd))
        self.es_distance_cnt = CS.es_distance_msg["Counter"]

      if self.es_lkas_cnt != CS.es_lkas_msg["Counter"]:
        can_sends.append(subarucan.create_es_lkas(self.sng_acc_resume, self.packer, CS.es_lkas_msg, enabled, visual_alert, left_line, right_line, left_lane_depart, right_lane_depart))
        self.es_lkas_cnt = CS.es_lkas_msg["Counter"]
        
      if self.throttle_cnt != CS.throttle_msg["Counter"]:
        can_sends.append(subarucan.create_throttle(self.packer, CS.throttle_msg, throttle_cmd))
        self.throttle_cnt = CS.throttle_msg["Counter"]
        
      if self.es_status_2_cnt != CS.es_status_2_msg["Counter"]:
         can_sends.append(subarucan.create_es_status_2(self.packer, CS.es_status_2_msg))
         self.es_status_2_cnt = CS.es_status_2_msg["Counter"]
        
      if self.cruise_buttons_cnt != CS.sw_cruise_buttons_msg["Counter"]:
        can_sends.append(subarucan.create_cruise_buttons(self.packer, CS.sw_cruise_buttons_msg, cspeed_dn_cmd, cspeed_up_cmd))
        self.cruise_buttons_cnt = CS.sw_cruise_buttons_msg["Counter"]

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.p.STEER_MAX

    return new_actuators, can_sends

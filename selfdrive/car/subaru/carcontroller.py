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
    #self.es_lanecenter_cnt = -1
    #self.es_status_2_cnt = -1
    #self.es_unknown1_cnt = -1
    #self.es_new_tst_2_cnt = -1
    #self.es_lkas_master_cnt = -1
    #self.es_ss_state_cnt = -1
    self.es_lkas_fwd_cnt = -1
    #self.es_steerjp_cnt = -1
    #self.steering_torque_cnt = -1
    #self.brake_pedal_cnt = -1
    self.dashlights_cnt = -1
    self.cruise_buttons_cnt = -1
    self.es_dashstatus_cnt = -1
    self.throttle_cnt = -1
    self.brake_pedal_cnt = -1                                                 
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
    
    self.prev_speed = 0
    self.adjust_speed = False
    
    self.dn_button_press = False
    self.dn_button_press_cnt = -1
    
    self.up_button_press = False
    self.up_button_press_cnt = -1
    
    self.cruise_cancel_btn_prev = 0
    self.lkas_mode = 2      #Modes:   1: OFF  2:Stock ACC no LKAS Standby 3: Stock LKAS (white lines) 4: Openpilot (green lines)
    self.pr_lkas_mode = 1
    self.mchange = True

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, c, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, left_line, right_line, left_lane_depart, right_lane_depart, dragonconf): #hud_speed,

    can_sends = []


    # *** Set the LKAS mode ***
    if ((CS.cruise_cancel_btn == 0) and (self.cruise_cancel_btn_prev == 1)):
      # Change the mode
      if CS.cruis_lkas_btn < 1:
        if self.pr_lkas_mode == 1:
          self.lkas_mode = 3
        else:
          self.lkas_mode = 1
        self.pr_lkas_mode = self.lkas_mode
      else: 
        self.lkas_mode = 2 
      self.mchange = True        
      #if (self.lkas_mode == 4):
      #  self.lkas_mode = 1
      
    self.cruise_cancel_btn_prev = CS.cruise_cancel_btn
   
    if not (self.lkas_mode > 2):
      CS.steerout = self.lkas_mode  #Just to test   
   
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

      if not (self.lkas_mode == 3):
        apply_steer = 0
      
      if CS.CP.carFingerprint in PREGLOBAL_CARS:
        can_sends.append(subarucan.create_preglobal_steering_control(self.packer, apply_steer, frame, self.p.STEER_STEP))
      else:
        can_sends.append(subarucan.create_steering_control(self.packer, apply_steer, frame, self.p.STEER_STEP))          
      
          
      self.apply_steer_last = apply_steer
      
    
      if (self.lkas_mode == 3):
        if not (abs(apply_steer) == 0):
          CS.steerout = abs(apply_steer)
          #CS.steerout = abs(actuators.steeringAngleDeg)
        else:
          CS.steerout = self.lkas_mode
          
      
    
    # *** speed control - check once per 2 seconds - km/h***
    cspeed_dn_cmd = False
    cspeed_up_cmd = False
    
    if (frame % 100) == 0:
    
      new_speedlim = (int(((dragonconf.dpSpeedLimit * CV.MS_TO_KPH) + 2.5) / 5) * 5)
      new_tunrlim  = (int(((dragonconf.dpTurnSpeedLimit * CV.MS_TO_KPH) + 2.5) / 5) * 5)
      cur_setspeed = int(((CS.out.cruiseState.speed * CV.MS_TO_KPH) + 2.5) / 5) * 5
      new_cspeed = new_speedlim
      
      #if speed limit is undefined, and turn limit is less than set speed limit
      if ((new_speedlim == 0) and (new_tunrlim > 29) and (new_tunrlim < cur_setspeed)):
        new_cspeed = new_tunrlim
        self.adjust_speed = True
        
      #if speed limit is defined, and turn limit is less than set speed limit
      if ((new_speedlim > 29) and (new_tunrlim > 29) and (new_tunrlim < new_speedlim)):
        new_cspeed = new_tunrlim
        self.adjust_speed = True
        
      #if the speed limit is defined and the turn limit is larger
      if ((new_speedlim > 29) and (new_tunrlim > 29) and (new_tunrlim > new_speedlim)):
        new_cspeed = new_speedlim
        
      #if the speed limit is defined and the turn limit is not defined
      if ((new_speedlim > 29) and (new_tunrlim == 0) and (new_speedlim < cur_setspeed)):
        new_cspeed = new_speedlim
      
      CS.speed_limit = new_cspeed
          
      if ((CS.cruise_state == 0) and (c.enabled) and (CS.speed_limit!=0) and (CS.out.cruiseState.enabled > 0)):
                      
        new_cspeed = new_cspeed + 5
        
        if ((abs(cur_setspeed - new_cspeed) > 15) and (CS.speed_limit != 0) and (CS.speed_limit > 25)):
          self.adjust_speed = True
          
        if ((cur_setspeed == new_cspeed) or (CS.speed_limit == 0)):
          self.adjust_speed = False
        
        if ((cur_setspeed != new_cspeed) and (CS.speed_limit != 0) and (self.adjust_speed)):
          
          #Check down first - but to a miminum of 30 kph
          if ((new_cspeed > 29) and (new_cspeed < cur_setspeed)):
            self.dn_button_press = True  #press the down button
            
          else:
            #Check up more than 20 kph, but to a max of 131 kph
            if ((new_cspeed < 131) and (new_cspeed > (cur_setspeed + 0))):
              self.up_button_press = True  #press the up button


    # *** speed control - press the up or down buttons***

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
      self.sng_acc_resume = True       #JP test
      #self.up_button_press = True  #press the up button  #JP test
    self.prev_cruise_state = CS.cruise_state

    if self.sng_acc_resume:  #JP test
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
        can_sends.append(subarucan.create_es_lkas(self.lkas_mode, self.sng_acc_resume, self.packer, CS.es_lkas_msg, enabled, visual_alert, left_line, right_line, left_lane_depart, right_lane_depart, CS.cruis_lkas_btn, self.mchange))
        self.es_lkas_cnt = CS.es_lkas_msg["Counter"]
        
      if self.es_dashstatus_cnt != CS.es_dashstatus_msg["Counter"]:
        can_sends.append(subarucan.create_es_dashstatus(self.packer, CS.es_dashstatus_msg, self.lkas_mode))
        self.es_dashstatus_cnt = CS.es_dashstatus_msg["Counter"]
        
      if self.throttle_cnt != CS.throttle_msg["Counter"]:
        can_sends.append(subarucan.create_throttle(self.packer, CS.throttle_msg, throttle_cmd))
        self.throttle_cnt = CS.throttle_msg["Counter"]
        
      #if self.es_lanecenter_cnt != CS.es_lanecenter_msg["Counter"]:
      #  can_sends.append(subarucan.create_es_lanecenter(self.lkas_mode, enabled, self.packer, CS.es_lanecenter_msg))
      #  self.es_lanecenter_cnt = CS.es_lanecenter_msg["Counter"]   
      
      #*** JP test - ES_LKAS steer message forward ***  
      #if not (self.lkas_mode == 4):
      #  if self.es_lkas_steer_cnt != CS.es_lkas_steer_msg["Counter"]:
      #    can_sends.append(subarucan.create_es_lkas_steer(self.packer, CS.es_lkas_steer_msg))
      #    self.es_lkas_steer_cnt = CS.es_lkas_steer_msg["Counter"]
      
      # if self.es_status_2_cnt != CS.es_status_2_msg["Counter"]:
        # can_sends.append(subarucan.create_es_status_2(self.packer, CS.es_status_2_msg))
        # self.es_status_2_cnt = CS.es_status_2_msg["Counter"]

      #*** JP test - ES_LKAS_master message forward ***
      # if self.es_lkas_master_cnt != CS.es_lkas_master_msg["Counter"]:
        # can_sends.append(subarucan.create_es_lkas_master(self.packer, CS.es_lkas_master_msg))
        # self.es_lkas_master_cnt = CS.es_lkas_master_msg["Counter"]        
     
     
      # if self.es_new_tst_2_cnt != CS.es_new_tst_2_msg["Counter"]:
        # can_sends.append(subarucan.create_es_new_tst_2(self.packer, CS.es_new_tst_2_msg))
        # self.es_new_tst_2_cnt = CS.es_new_tst_2_msg["Counter"]

      if self.cruise_buttons_cnt != CS.sw_cruise_buttons_msg["Counter"]:
        can_sends.append(subarucan.create_cruise_buttons(self.packer, CS.sw_cruise_buttons_msg, cspeed_dn_cmd, cspeed_up_cmd))
        self.cruise_buttons_cnt = CS.sw_cruise_buttons_msg["Counter"]
        
      #*** JP test - ES_STEER_JP steer message forward ***  
      # if self.es_steerjp_cnt != CS.es_steerjp_msg["Counter"]:
        # can_sends.append(subarucan.create_es_steerjp(self.lkas_mode, enabled, actuators.steeringAngleDeg, self.apply_steer_last, self.packer, CS.es_steerjp_msg))
        # self.es_steerjp_cnt = CS.es_steerjp_msg["Counter"]  
      
      # if self.steering_torque_cnt != CS.steering_torque_msg["Counter"]:
        # can_sends.append(subarucan.create_steering_torque(self.packer, CS.steering_torque_msg))
        # self.steering_torque_cnt = CS.steering_torque_msg["Counter"]
        
      
      #*** JP test - es_unknown1 message forward ***
      #if self.es_unknown1_cnt != CS.es_unknown1_msg["Counter"]:
      #  can_sends.append(subarucan.create_es_unknown1(self.packer, CS.es_unknown1_msg))
      #  self.es_unknown1_cnt = CS.es_unknown1_msg["Counter"]   
        
      #*** JP test - Start_Stop_State message forward ***
      #if self.es_ss_state_cnt != CS.sw_ss_state_msg["Counter"]:
      #  can_sends.append(subarucan.create_ss_state(self.packer, CS.sw_ss_state_msg))
      #  self.es_ss_state_cnt = CS.sw_ss_state_msg["Counter"]   
        
      
      #if self.dashlights_cnt != CS.dashlights_msg["Counter"]:
      #  can_sends.append(subarucan.create_dashlights(self.packer, CS.dashlights_msg))
      #  self.dashlights_cnt = CS.dashlights_msg["Counter"]
      
      # if self.brake_pedal_cnt != CS.brake_pedal_msg["Counter"]:
        # can_sends.append(subarucan.create_brake_pedal(self.packer, CS.brake_pedal_msg, speed_cmd, pcm_cancel_cmd))
        # self.brake_pedal_cnt = CS.brake_pedal_msg["Counter"]
    
    self.mchange = False
    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / self.p.STEER_MAX

    return new_actuators, can_sends

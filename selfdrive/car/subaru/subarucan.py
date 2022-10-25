import copy
from cereal import car

VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_steering_control(packer, apply_steer, frame, steer_step):

  idx = (frame / steer_step) % 16

  values = {
    "Counter": idx,
    "LKAS_Output": apply_steer ,
    "LKAS_Request": 1 if apply_steer != 0 else 0,
    "SET_1": 1
    #"SET_2": 0  #US ES
  }

  #Check possible return on CAN2 also
    
  return packer.make_can_msg("ES_LKAS", 0, values)
  
  
#def create_steering_control2(packer, apply_steer, frame, steer_step):
#
#  idx = (frame / steer_step) % 16
#
#  values = {
#    "Counter": idx,
#    "LKAS_Output": apply_steer,
#    "LKAS_Request": 1 if apply_steer != 0 else 0,
#    "SET_1": 1,
#    "SET_2": 0
#  }

  #Check possible return on CAN2 also
    
#  return packer.make_can_msg("ES_LKAS", 2, values)

# def create_Engine_Stop_Start(packer, frame, steer_step):

  # #idx = frame / steer_step) % 16
  # idx = frame % 16

  # values = {
    # "Counter": idx,
    # "SIG_1" : 1,
    # "STOP_START_STATE": 2
  # }

  # #Check possible return on CAN2 also
  # #packer.make_can_msg("Engine_Stop_Start", 2, values)  
  # return packer.make_can_msg("Engine_Stop_Start", 0, values)
  
def create_Engine_Stop_Start2(packer, frame, steer_step):

  #idx = frame / steer_step) % 16
  idx = frame % 16

  values = {
    "Counter": idx,
    "SIG_1" : 1,
    "STOP_START_STATE": 2
  }

  #Check possible return on CAN2 also
  #packer.make_can_msg("Engine_Stop_Start", 2, values)  
  return packer.make_can_msg("Engine_Stop_Start", 2, values)
  
#def create_Stop_Start(packer, frame, steer_step):

#  idx = (frame / steer_step) % 16

#  values = {
#    "Counter": idx
#  }

  #Check possible return on CAN2 also
  #packer.make_can_msg("Engine_Stop_Start", 2, values)  
#  return packer.make_can_msg("STOP_START", 0, values)

def create_steering_status(packer, apply_steer, frame, steer_step):
  return packer.make_can_msg("ES_LKAS_State", 0, {})

def create_es_distance(packer, es_distance_msg, pcm_cancel_cmd, cspeed_dn_cmd, cspeed_up_cmd):

  values = copy.copy(es_distance_msg)
  if pcm_cancel_cmd:
    values["Cruise_Cancel"] = 1
    
  # Enable LKAS for market specific models
  #US ES#values["Signal1"] = 5
  
  if ((cspeed_dn_cmd == True) and (cspeed_up_cmd == False)):
    values["Cruise_Set"] = 1    #JP Test
    
  if ((cspeed_up_cmd == True) and (cspeed_dn_cmd == False)):
    values["Cruise_Resume"] = 1
    
  #values["Signal1"] = 1     #in search of LKAS_MASTER

  return packer.make_can_msg("ES_Distance", 0, values)

def create_es_lkas(lkas_mode, sng_acc_resume, packer, es_lkas_msg, enabled, visual_alert, left_line, right_line, left_lane_depart, right_lane_depart):

  values = copy.copy(es_lkas_msg)

  # Filter the stock LKAS "Keep hands on wheel" alert
  if values["LKAS_Alert_Msg"] == 1:
    values["LKAS_Alert_Msg"] = 0

  # Filter the stock LKAS sending an audible alert when it turns off LKAS
  if values["LKAS_Alert"] == 27:
    values["LKAS_Alert"] = 0

  # Filter the stock LKAS sending an audible alert when "Keep hands on wheel" alert is active (2020+ models)
  if values["LKAS_Alert"] == 28 and values["LKAS_Alert_Msg"] == 7:
    values["LKAS_Alert"] = 0
  
  # Filter the stock LKAS sending an audible alert when "Keep hands on wheel OFF" alert is active (2020+ models)
  if values["LKAS_Alert"] == 30:
    values["LKAS_Alert"] = 0

  # Filter the stock LKAS sending "Keep hands on wheel OFF" alert (2020+ models)
  if values["LKAS_Alert_Msg"] == 7:
    values["LKAS_Alert_Msg"] = 0
    
  # Show Keep hands on wheel alert for openpilot steerRequired alert
  if (lkas_mode == 4):
    if visual_alert == VisualAlert.steerRequired:
      values["LKAS_Alert_Msg"] = 1

  # Ensure we don't overwrite potentially more important alerts from stock (e.g. FCW)
  if visual_alert == VisualAlert.ldw and values["LKAS_Alert"] == 0:
    if left_lane_depart:
      values["LKAS_Alert"] = 12 # Left lane departure dash alert
    elif right_lane_depart:
      values["LKAS_Alert"] = 11 # Right lane departure dash alert

  # Aus values["LKAS_ACTIVE"] = 1 from start
  #lkas_lanes = 0
  #if (lkas_mode == 4):
  #  lkas_lanes = 2
  #else:
  #  lkas_lanes = 1
  
  if (lkas_mode == 4):
    values["LKAS_ACTIVE"] = 1 # Try to get LKAS to stop
    values["LKAS_Enable_1"] = 0  # Try to get LKAS to stop
    values["LKAS_Dash_State"] = 1 # Try to get LKAS to stop
    values["Signal5"] = 0    #JP Test Set to Wilderness
  else:
    values["LKAS_ACTIVE"] = 1 # Show LKAS lane lines
    values["LKAS_Enable_1"] = 1  # Set to Wilderness - TO BE CHECKED
    values["LKAS_Dash_State"] = 1 # White enabled indicator  -- JP Test
    values["Signal5"] = 0    #JP Test Set to Wilderness
  
  if enabled:
  
    if (lkas_mode == 3):
      values["LKAS_ACTIVE"] = 1 # Try to get LKAS to stop
      values["LKAS_Enable_1"] = 1  # Try to get LKAS to stop
      values["LKAS_Dash_State"] = 2 # Try to get LKAS to stop
      values["Signal5"] = 0    #JP Test Set to Wilderness
    else:
      values["LKAS_ACTIVE"] = 1 # Show LKAS lane lines
      values["LKAS_Dash_State"] = 2 # Green enabled indicator
      values["Signal5"] = 16    #JP Test Set to Wilderness
      values["LKAS_Enable_1"] = 1  # - Set to Wilderness - TO BE CHECKED
    
    values["LKAS_Left_Line_Enable"] = 1 #1 Show LKAS left lane line
    values["LKAS_Right_Line_Enable"] = 1 #1 Show LKAS right line
    
    if sng_acc_resume:
      values["LKAS_Left_Line_Enable"] = 0 # Show LKAS left lane line
      values["LKAS_Right_Line_Enable"] = 0 # Show LKAS right line
    else:
      # Enable lines only with certain modes:
      if (lkas_mode == 4):
        values["LKAS_Left_Line_Enable"] = int(left_line) #1 Show LKAS left lane line
        values["LKAS_Right_Line_Enable"] = int(right_line) #1 Show LKAS right line
        values["LKAS_Left_Line_Visible"] = int(left_line) * 2     #int(left_line)
        values["LKAS_Right_Line_Visible"] = int(right_line) * 2     #int(right_line)
      if (lkas_mode == 3):
        values["LKAS_Left_Line_Visible"] = int(left_line)     #int(left_line)
        values["LKAS_Right_Line_Visible"] = int(right_line)      #int(right_line)
  
    if (lkas_mode < 3):
      values["LKAS_Left_Line_Visible"] = 1    #int(left_line)
      values["LKAS_Right_Line_Visible"] = 1      #int(right_line)
  
  else:
     #values["LKAS_Dash_State"] = 0 # LKAS Not enabled  -- JP Test
    values["LKAS_Left_Line_Enable"] = 1 # Enable LKAS left lane line
    values["LKAS_Right_Line_Enable"] = 1 # Enable LKAS right line 
    values["LKAS_Left_Line_Visible"] = 0     #int(left_line)
    values["LKAS_Right_Line_Visible"] = 0      #int(right_line)
  
  # Enable LKAS for market specific models

  values["Signal1"] = 0    #JP Test - 7
  values["Signal2"] = 0    #JP Test
  #values["Signal5"] = 1    #JP Test
   
  #values["LKAS_Enable_2"] = 3  #ACC Acceleration Characteristics - defuault 1 - CHECKED ACC_ACEL_CHAR
  values["Signal3"] = 128    # relates to RAB setting 128 worked - Wilderness 144 - CHECKED
  
  values["LEFT_RIGHT_SET"] = 1      #JP Test - SETTINGS: LANE_CENTRE_SET:(bt50) LEFT_RIGHT_SET:(bt51) LKAS_SET:(bt55) -> (bt53), (bt52), (bt49), (bt48), (bt47) does nothing noticeable 
  if (lkas_mode == 4):
    values["LKAS_SET"] = 1
    #values["NEW_SIGNAL_2"] = 0
  else:
    values["LKAS_SET"] = 1
    
  if (lkas_mode < 3):
    values["LANE_CENTRE_SET"] = 1
  else:
    values["LANE_CENTRE_SET"] = 0   

  if (lkas_mode == 3):
    values["LKAS_SET"] = 1    
    values["LANE_CENTRE_SET"] = 0
  # *** TEST - SEARCH FOR WARNING SETTINGS ***
  #values["NEW_SIGNAL_1"] = 0
  #values["NEW_SIGNAL_2"] = 0
  
  # Enable LKAS for market specific models - try here
  packer.make_can_msg("ES_LKAS_State", 2, values)
  return packer.make_can_msg("ES_LKAS_State", 0, values)

def create_es_dashstatus(packer, dashstatus_msg):
  values = copy.copy(dashstatus_msg)

  # Filter stock LKAS disabled and Keep hands on steering wheel OFF alerts
  if values["LKAS_State_Msg"] in [2, 3]:
    values["LKAS_State_Msg"] = 0      #JP Test - try 1 here
    
  #values["LKAS_State_Msg"] = 0  # (Check again) JP tried setting to 0 continously - no effect
  
  # Enable LKAS for market specific models
  values["Signal1"] = 0        #JP Test - try 1 here - change to 0 as Wilderness
  values["Signal2"] = 0        #JP Test - try 1 here - change to 0 as Wilderness
  values["Signal5"] = 2        #JP Test - try 2 as Wilderness  
  values["Signal7"] = 0        #JP Test - try 2 as Wilderness  
  
  values["Signal3"] = 1        #LKAS_dash icon ON

  return packer.make_can_msg("ES_DashStatus", 0, values)
  
def create_es_lanecenter(lkas_mode, enabled, packer, es_lanecenter_msg):
  values = copy.copy(es_lanecenter_msg)

  # Enable LKAS for market specific models - Blue lines and blue following car
  # Find the correct variable
  #if (lkas_mode == 4):          #4: Openpilot (green lines)
  values["Dash_Display"] = 0  
  values["Signal2"] = 0     #0 JP Test carefull
  values["Signal3"] = 0     #15 JP Test carefull
  
  if (enabled):  
    if (lkas_mode == 1):          # 1: Stock LKAS + lane centering  
      values["Dash_Display"] = 6     #7 JP Test carefull - switched off for OP test
      values["Signal2"] = 0     #0 JP Test carefull
      values["Signal3"] = 15     #15 JP Test carefull
    
    if (lkas_mode == 2):          # 2:Stock LKAS + lane centering + vehicle follow
      values["Dash_Display"] = 7     #7 JP Test carefull - switched off for OP test
      values["Signal2"] = 0     #0 JP Test carefull
      values["Signal3"] = 15     #15 JP Test carefull
    
    #if (lkas_mode == 3):      # 3: Stock LKAS (white lines)
    #  values["Dash_Display"] = 0     #7 JP Test carefull - switched off for OP test
    #  values["Signal2"] = 0     #0 JP Test carefull
    #  values["Signal3"] = 0     #15 JP Test carefull

  return packer.make_can_msg("ES_LANE_CENTER", 0, values)
  
# def create_es_lkas_steer(packer, es_lkas_steer_msg):
  # values = copy.copy(es_lkas_steer_msg)

  # # Enable LKAS for market specific models
  # # Find the correct variable
  
  # values["SET_1"] = 0     #JP alligned to Wilderness  - CHECKED
  # values["SET_2"] = 1     #JP alligned to Wilderness  - CHECKED

  # return packer.make_can_msg("ES_LKAS", 0, values)
  
def create_es_steerjp(lkas_mode, enabled, steer_angle, apply_steer, packer, es_steerjp_msg):
  values = copy.copy(es_steerjp_msg)

  # Enable LKAS for market specific models
  # Find the correct variable
  #Trying to stop the stock steering
  
  #values["NEW_SIGNAL_1"] = 0 
  
  #if (lkas_mode > 2):
    #values["STEER_STEP"] = 0
  if (enabled):  
    if (lkas_mode == 4):
      values["NEW_SIGNAL_1"] = 3
      values["STEER_ANGLE"] = steer_angle
      values["STEER_OUTPUT"] = apply_steer * 0.1  #JP alligned to Wilderness  - TO BE CHECKED steer_angle * 0.5 * (0.075 * speed + 0.55)  
  
    if (lkas_mode < 3):
      values["NEW_SIGNAL_1"] = 3     #JP alligned to Wilderness  - TO BE CHECKED
      values["STEER_OUTPUT"] = values["STEER_ANGLE"] * 2     #JP alligned to Wilderness  - TO BE CHECKED
  
  else:
    values["NEW_SIGNAL_1"] = 3     #JP alligned to Wilderness  - TO BE CHECKED
    values["STEER_OUTPUT"] = values["STEER_ANGLE"] * 1     #JP alligned to Wilderness  - TO BE CHECKED

  #packer.make_can_msg("ES_STEER_JP", 2, values)
  return packer.make_can_msg("ES_STEER_JP", 0, values)

# def create_es_status_2(packer, es_status_2_msg):
  # values = copy.copy(es_status_2_msg)

  # # Enable LKAS for market specific models RAB
  # # Find the correct variable
  # values["Signal1"] = 7     #JP Test carefull 7    Aus  12
  # values["Signal2"] = 0     #JP Test carefull  0    Aus 3
  # values["Signal3"] = 12     #JP Test carefull 12   Aus 12/14

  # return packer.make_can_msg("ES_Status_2", 0, values)
  
# def create_es_new_tst_2(packer, es_new_tst_2_msg):
  # values = copy.copy(es_new_tst_2_msg)

  # # Find the correct variable
  # values["NEW_SIGNAL_1"] = 64     #JP alligned to Wilderness  - TO BE CHECKED  64 alligns to crosstrek set value
 

  # return packer.make_can_msg("NEW_TST_2", 0, values)
  
# def create_es_lkas_master(packer, es_lkas_master_msg):
  # values = copy.copy(es_lkas_master_msg)

  # values["Signal1"] = 1     #JP Test carefull 7    Aus  1

  # return packer.make_can_msg("ES_LKAS_Master", 0, values)
  
# def create_es_unknown1(packer, es_unknown1_msg):
  # values = copy.copy(es_unknown1_msg)

  # values["Signal1"] = 1     #JP Test carefull 7    Aus  1

  # return packer.make_can_msg("ES_UNKNOWN1", 1, values)

# def create_ss_state(packer, es_ss_state_msg):
  # values = copy.copy(es_ss_state_msg)

  # values["Signal1"] = 92     #JP Test SA 92    Aus  95 - May be related to fuel octane

  # return packer.make_can_msg("START_STOP_STATE", 0, values)

def create_cruise_buttons(packer, sw_cruise_buttons_msg, cspeed_dn_cmd, cspeed_up_cmd):
  values = copy.copy(sw_cruise_buttons_msg)

  # Find the correct variable
  values["Signal2"] = 0
  
  if ((cspeed_dn_cmd == True) and (cspeed_up_cmd == False)):
    values["Set"] = 1    #JP Test
    
  if ((cspeed_up_cmd == True) and (cspeed_dn_cmd == False)):
    values["Resume"] = 1
    
  return packer.make_can_msg("Cruise_Buttons", 0, values)
  
# def create_steering_torque(packer, steering_torque_msg):
  # values = copy.copy(steering_torque_msg)
  
  # values["Signal1"] = 1     #JP alligned to Wilderness  - TO BE CHECKED
  
  # return packer.make_can_msg("Steering_Torque", 0, values)

def create_dashlights(packer, dashlights_msg):
  values = copy.copy(dashlights_msg)
  
  #values["STOP_START"] = 1     #JP alligned to Wilderness  - TO BE CHECKED
  
  return packer.make_can_msg("Dashlights", 0, values)

def create_throttle(packer, throttle_msg, throttle_cmd):

  values = copy.copy(throttle_msg)
  if throttle_cmd:
    values["Throttle_Pedal"] = 5

  return packer.make_can_msg("Throttle", 2, values)

# def create_brake_pedal(packer, brake_pedal_msg, speed_cmd, brake_cmd):

  # values = copy.copy(brake_pedal_msg)
  # if speed_cmd:
    # values["Speed"] = 3
  # if brake_cmd:
    # values["Brake_Pedal"] = 5
    # values["Brake_Lights"] = 1

  # return packer.make_can_msg("Brake_Pedal", 2, values)
  
# *** Subaru Pre-global ***

def subaru_preglobal_checksum(packer, values, addr):
  dat = packer.make_can_msg(addr, 0, values)[2]
  return (sum(dat[:7])) % 256

def create_preglobal_steering_control(packer, apply_steer, frame, steer_step):

  idx = (frame / steer_step) % 8

  values = {
    "Counter": idx,
    "LKAS_Command": apply_steer,
    "LKAS_Active": 1 if apply_steer != 0 else 0
  }
  values["Checksum"] = subaru_preglobal_checksum(packer, values, "ES_LKAS")

  return packer.make_can_msg("ES_LKAS", 0, values)

def create_preglobal_es_distance(packer, cruise_button, es_distance_msg):

  values = copy.copy(es_distance_msg)
  values["Cruise_Button"] = cruise_button

  values["Checksum"] = subaru_preglobal_checksum(packer, values, "ES_Distance")

  return packer.make_can_msg("ES_Distance", 0, values)

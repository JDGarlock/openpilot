from cereal import car
import numpy as np
from common.numpy_fast import interp, clip
from selfdrive.car import make_can_msg
from selfdrive.car.ford.fordcan import create_steer_command, create_speed_command, create_speed_command2, create_ds_118, create_lkas_ui, spam_cancel_button
from opendbc.can.packer import CANPacker


MAX_STEER_DELTA = 0.2
TOGGLE_DEBUG = False
COUNTER_MAX = 7

ANGLE_MAX_BP = [0., 27., 36.]
ANGLE_MAX_V = [410., 92., 36.]

ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .15]     #windup
ANGLE_DELTA_VU = [5., 3.5, 0.4] #unwind

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.packer = CANPacker(dbc_name)
    self.enable_camera = CP.enableCamera
    self.enabled_last = False
    self.main_on_last = False
    self.vehicle_model = VM
    self.generic_toggle_last = 0
    self.steer_alert_last = False
    self.lkas_action = 0
    #self.lkasToggle = 1
    self.lastAngle = 0
    self.angleReq = 0
    self.sappConfig = 0
    self.sappChime = 0
    self.chimeCounter = 0
    self.sappConfig_last = 0
    self.angleReq_last = 0
    self.apaCounter = 0
    self.sappAction = 0
    self.eightysix = 0
    self.cs514_cnt_cntr = 0
    self.alwaysTrue = True
    self.cs514_cnt_cntr_last = 0
    self.cs1045_cnt_cntr = 0
    self.cs1045_cnt_cntr_last = 0
  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):

    can_sends = []
    steer_alert = visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired
    apply_steer = actuators.steerAngle
    self.cs514_cnt_cntr_last = self.cs514_cnt_cntr
    self.cs1045_cnt_cntr_last = self.cs1045_cnt_cntr
    self.cs1045_cnt_cntr = CS.actlbrknocnt
    if self.enable_camera:
      if CS.epsAssistLimited:
        print("PSCM Assist Limited")
      if (frame % 2) == 0:
        if self.alwaysTrue == True:
          #  can_sends.append(create_speed_command(self.packer, CS.vehSpeed, CS.trlraid, CS.actlnocs, CS.actlnocnt, CS.actlqf, CS.epsgear))
          #can_sends.append(create_speed_command2(self.packer, CS.vehSpeed2, CS.lsmcdecel, CS.actlbrknocs, CS.actlbrknocnt, CS.actlbrkqf))
          self.drvstate = 6
          can_sends.append(create_ds_118(self.packer,  CS.filler1, CS.filler2, CS.filler3, CS.brakectr, CS.awdlckmax, CS.awdlckmn, self.drvstate, CS.drvtq, CS.emergbrk, CS.stoplmp, CS.angle))
          #I'm too lazy to calculate this counter since it is gonna be used with 0 speed
          #514 Counter/Checksum/Speed
          if self.cs1045_cnt_cntr_last == 0:
            can_sends.append(make_can_msg(0x202, b'\x04\xf3\x48\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xc0\xfc\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 1:
            can_sends.append(make_can_msg(0x202, b'\x04\xf1\x58\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xc4\xfb\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 2:
            can_sends.append(make_can_msg(0x202, b'\x04\xef\x68\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xc8\xfa\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 3:
            can_sends.append(make_can_msg(0x202, b'\x04\xed\x78\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xcc\xf9\x80\x00\x00\x00', 2))     
          if self.cs1045_cnt_cntr_last == 4:
            can_sends.append(make_can_msg(0x202, b'\x04\xfb\x08\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xd0\xf8\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 5:
            can_sends.append(make_can_msg(0x202, b'\x04\xf9\x18\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xd4\xf7\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 6:
            can_sends.append(make_can_msg(0x202, b'\x04\xf7\x28\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xd8\xf6\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 7:
            can_sends.append(make_can_msg(0x202, b'\x04\xf5\x38\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xdc\xf5\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 8:
            can_sends.append(make_can_msg(0x202, b'\x04\xf3\x48\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xe0\xf4\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 9:
            can_sends.append(make_can_msg(0x202, b'\x04\xf1\x58\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xe4\xf3\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 10:
            can_sends.append(make_can_msg(0x202, b'\x04\xef\x68\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xe8\xf2\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 11:
            can_sends.append(make_can_msg(0x202, b'\x04\xed\x78\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xec\xf1\x80\x00\x00\x00', 2))        
          if self.cs1045_cnt_cntr_last == 12:
            can_sends.append(make_can_msg(0x202, b'\x04\xfb\x08\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xf0\xf0\x80\x00\x00\x00', 2)) 
          if self.cs1045_cnt_cntr_last == 13:
            can_sends.append(make_can_msg(0x202, b'\x04\xf9\x18\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xf4\xef\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 14:
            can_sends.append(make_can_msg(0x202, b'\x04\xf7\x28\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xf8\xee\x80\x00\x00\x00', 2))
          if self.cs1045_cnt_cntr_last == 15:
            can_sends.append(make_can_msg(0x202, b'\x04\xf5\x38\x00\x60\x6e\x00\x00', 2))
            can_sends.append(make_can_msg(0x415, b'\x00\x00\xfc\xed\x80\x00\x00\x00', 2))
      if pcm_cancel:
       #print("CANCELING!!!!")
        can_sends.append(spam_cancel_button(self.packer))
      if (frame % 1) == 0:
        self.main_on_last = CS.out.cruiseState.available
      #SAPP Config Value Handshake
      if (frame % 2) == 0:
        if not enabled:
          self.apaCounter = 0
          self.eightysix = 0
          self.angleReq = 0
          self.sappAction = 0
        if enabled:
          self.apaCounter += 1 #Increment counter 
          #Sets config to base value when init handshake
          if CS.sappHandshake == 0 and self.sappConfig_last not in [16, 86, 224] :
            self.sappConfig = 70
          #waits for the pscm to respond, and waits 8 frames as well. sets config to response
          if CS.sappHandshake == 1 and self.apaCounter > 8:
            self.sappConfig = 86
            self.eightysix += 1
          #waits 5 frames then sends the angle request
          if CS.sappHandshake == 1 and self.apaCounter > 13 and self.sappConfig_last == 86:
            self.angleReq = 1
          #when 20 frames have passed at response config, values are cleared and angle request is held
          if self.sappConfig_last == 86 and self.eightysix == 20:
            self.apaCounter = 0
            self.eightysix = 0
            self.angleReq = 1
          #pscm responds to handshake. config is set to parallel action. 
          if CS.sappHandshake == 2 and self.sappConfig_last != 16: # and self.apaCounter in range (15,16):
            self.sappConfig = 224
            self.angleReq = 1
            self.sappAction += 1
          #once action is held for 3 frames, final response is sent. pscm is handshaken
          if CS.sappHandshake == 2 and self.sappAction >= 3 and self.sappConfig_last == 224:
            self.sappConfig = 16
            self.angleReq = 1
          #if pscm faults, values reset to retry. 
          if CS.sappHandshake == 3:
            self.sappConfig = 0
            self.apaCounter = 0
            self.angleReq = 0
        self.sappConfig_last = self.sappConfig
        self.angleReq_last = self.angleReq
        print("Handshake:", CS.sappHandshake, "Config:", self.sappConfig_last, "Counter:", self.apaCounter, "AngleRequest:", self.angleReq, "fwdAction:", self.sappAction)
        self.lkas_action = 0 #6 Finished 5 NotAccessible 4 ApaCancelled 2 On 1 Off  
        angle_lim = interp(CS.out.vEgo, ANGLE_MAX_BP, ANGLE_MAX_V)
        apply_steer = clip(apply_steer, -angle_lim, angle_lim)
        if enabled:
          if self.lastAngle * apply_steer > 0. and abs(apply_steer) > abs(self.lastAngle):
            angle_rate_lim = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
          else:
            angle_rate_lim = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
          
          apply_steer = clip(apply_steer, self.lastAngle - angle_rate_lim, self.lastAngle + angle_rate_lim) 
        else:
          apply_steer = CS.out.steeringAngle
        self.lastAngle = apply_steer
        can_sends.append(create_steer_command(self.packer, apply_steer, enabled, CS.out.steeringAngle, self.lkas_action, self.angleReq_last, self.sappConfig_last, self.sappChime))
        self.generic_toggle_last = CS.out.genericToggle
      if (frame % 1) == 0 or (self.enabled_last != enabled) or (self.main_on_last != CS.out.cruiseState.available) or (self.steer_alert_last != steer_alert):
        if steer_alert:
          self.steer_chime = 2
        else:
          self.steer_chime = 0
        can_sends.append(create_lkas_ui(self.packer, CS.out.cruiseState.available, enabled, self.steer_chime, CS.ipmaHeater, CS.ahbcCommanded, CS.ahbcRamping, CS.ipmaConfig, CS.ipmaNo, CS.ipmaStats, CS.persipma, CS.dasdsply, CS.x30))
        self.enabled_last = enabled                         
      self.steer_alert_last = steer_alert

    return can_sends

from cereal import car
import numpy as np
from selfdrive.car.ford.fordcan import create_steer_command, create_lkas_ui, spam_cancel_button
from opendbc.can.packer import CANPacker


MAX_STEER_DELTA = 1
TOGGLE_DEBUG = False
COUNTER_MAX = 7

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
    self.lkasCounter = 0

  def update(self, enabled, CS, frame, actuators, visual_alert, pcm_cancel):

    can_sends = []
    steer_alert = visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired

    apply_steer = actuators.steer
    if (frame % 100) == 0:
      self.lkasCounter +=1 
    if self.enable_camera:

      if pcm_cancel:
       print("CANCELING!!!!")
       can_sends.append(spam_cancel_button(self.packer))

      if (frame % 3) == 0:
      #Stock IPMA Message is 33Hz. PSCM accepts commands at max 44Hz. 
        curvature = self.vehicle_model.calc_curvature(actuators.steerAngle*np.pi/180., CS.out.vEgo)
        self.lkas_action = 5 #6 Finished 5 NotAccessible 4 ApaCancelled 2 On 1 Off  
        #if self.lkasCounter < COUNTER_MAX:
        #  can_sends.append(create_steer_command(self.packer, apply_steer, enabled, CS.lkas_state, CS.out.steeringAngle, curvature, self.lkas_action))
        #else:
        #  self.lkasCounter = 0
        #  print("CAN Message successfully blocked for 1 message")
        #  pass
        can_sends.append(create_steer_command(self.packer, apply_steer, enabled, CS.out.steeringAngle, self.lkas_action))
        self.generic_toggle_last = CS.out.genericToggle
      if (frame % 1) == 0 or (self.enabled_last != enabled) or (self.main_on_last != CS.out.cruiseState.available) or (self.steer_alert_last != steer_alert):
        can_sends.append(create_lkas_ui(self.packer, CS.out.cruiseState.available, enabled, steer_alert, CS.ipmaHeater, CS.ahbcCommanded, CS.ahbcRamping, CS.ipmaConfig, CS.ipmaNo, CS.ipmaStats))
        self.enabled_last = enabled                         
        self.main_on_last = CS.out.cruiseState.available
      self.steer_alert_last = steer_alert

    return can_sends

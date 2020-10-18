from common.numpy_fast import clip
from selfdrive.car.ford.values import MAX_ANGLE

def create_steer_command(packer, angle_cmd, enabled, angle_steers, lkas_action, angleReq, sappConfig, sappChime):
  """Creates a CAN message for the Ford Steer Command."""

  #if enabled and lkas available:
  if enabled: # and (frame % 500) >= 3:
    action = lkas_action
    angle_cmd = angle_steers/MAX_ANGLE
  else:
    action = 0xf
    angle_cmd = angle_steers/MAX_ANGLE

  angle_cmd = clip(angle_cmd * MAX_ANGLE, - MAX_ANGLE, MAX_ANGLE)

  values = {
    "ApaSys_D_Stat": action,
    "EPASExtAngleStatReq": angleReq,
    "ExtSteeringAngleReq2": angle_cmd,
    "SAPPStatusCoding": sappConfig,
    "ApaChime_D_Rq": sappChime,
  }
  return packer.make_can_msg("ParkAid_Data", 2, values)

def create_speed_command(packer, speed, trlraid, actlnocs, actlnocnt, actlqf, gear):
  """Creates a CAN message for the Ford Speed Command."""
  values = {
    "VehVTrlrAid_B_Avail": trlraid,
    "VehVActlEng_No_Cs": actlnocs,
    "VehVActlEng_No_Cnt": actlnocnt,
    "VehVActlEng_D_Qf": actlqf,
    "GearRvrse_D_Actl": gear,
    "Veh_V_ActlEng": speed,
  }
  return packer.make_can_msg("EngVehicleSpThrottle2", 2, values)

def create_speed_command2(packer, speed2, longcomp, latcomp, yawcomp):
  """Creates a CAN message for the Ford Speed Command."""
  values = {
    "VehOverGnd_V_Est": speed2,
    "VehLongComp_A_Actl": longcomp,
    "VehLatComp_A_Actl": latcomp,
    "VehYawComp_W_Actl": yawcomp,
  }
  return packer.make_can_msg("BrakeSnData_3", 2, values)

def create_speed_command3(packer, speed3, lsmcdecel, actlbrknocs, actlbrknocnt, actlbrkqf):
  """Creates a CAN message for the Ford Speed Command."""
  values = {
    "Veh_V_ActlBrk": speed3,
    "LsmcBrkDecel_D_Stat": lsmcdecel,
    "VehVActlBrk_No_Cs": actlbrknocs,
    "VehVActlBrk_No_Cnt": actlbrknocnt,
    "VehVActlBrk_D_Qf": actlbrkqf,
  }
  return packer.make_can_msg("BrakeSysFeatures", 2, values)

def create_lkas_ui(packer, main_on, enabled, steer_alert, defog, ahbc, ahbcramping, config, noipma, stats, persipma, dasdsply):
  """Creates a CAN message for the Ford Steer Ui."""
  if enabled:
    lines = 0x6
  else:
    lines = 0xc

  values = {
    "PersIndexIpma_D_Actl": persipma,
    "DasStats_D_Dsply": dasdsply,
    "Set_Me_X30": 0x30,
    "Lines_Hud": lines,
    "Hands_Warning_W_Chime": steer_alert,
    "CamraDefog_B_Req": defog,
    "AhbHiBeam_D_Rq": ahbc,
    "AhbcRampingV_D_Rq": ahbcramping,
    "FeatConfigIpmaActl": config,
    "FeatNoIpmaActl": noipma,
    "CamraStats_D_Dsply": stats,
  }
  return packer.make_can_msg("Lane_Keep_Assist_Ui", 0, values)

def spam_cancel_button(packer):
  values = {
    "Cancel": 1
  }
  return packer.make_can_msg("Steering_Buttons", 0, values)

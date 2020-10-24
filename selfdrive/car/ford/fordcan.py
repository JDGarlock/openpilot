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

def create_ds_118(packer, filler1, filler2, filler3, brakectr, awdlckmax, awdlckmn, drvstate, drvtq, emergbrk, stoplmp, angle):
  """Creates a CAN message for the Ford 118 message."""

  values = {
    "BrkCtrFnd_B_Stat"  brakectr,
    "AwdLck_Tq_RqMx": awdlckmax,
    "AwdLck_Tq_RqMn": awdlckmn,
    "DrvSte_D_Stat": drvstate,
    "DrvSte_Tq_Rq": drvtq,
    "EmgcyBrkLamp_D_Rq": emergbrk,
    "StopLamp_B_RqBrk": stoplmp,
    "SteWhlRelInit_An_Sns": angle,
    "DS_Filler_1": filler1,
    "DS_Filler_2": filler2,
    "DS_Filler_3": filler3,
  }
  return packer.make_can_msg("BrakeSnData_5", 0, values)

def create_lkas_ui(packer, main_on, enabled, steer_alert, defog, ahbc, ahbcramping, config, noipma, stats, persipma, dasdsply, x30):
  """Creates a CAN message for the Ford Steer Ui."""
  if enabled:
    lines = 0x6
  else:
    lines = 0xc

  values = {
    "PersIndexIpma_D_Actl": persipma,
    "DasStats_D_Dsply": dasdsply,
    "Set_Me_X30": x30,
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

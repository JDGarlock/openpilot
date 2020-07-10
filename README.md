Ford fork of Openpilot. 

**IMPORTANT!!! PLEASE READ THE FOLLOWING INFORMATION BELOW!!!**

# Important Information

This brand currently has limited openpilot support on the F150 and Fusion only through community maintained branches. Safety code is not working properly and should **NOT** be used without fully understanding the ramifications of such. 

Ford currently has a steering lockout on the PSCM, and after 10 seconds, commands will drop for approximately 200-300ms. A fix has not yet been implemented. 

# Supported Vehicles

| Make      | Model                         | Supported Package | ACC              | No ACC accel below | No LKA below | No Lateral below |
| ----------| ------------------------------| ------------------| -----------------| -------------------| -------------| -----------------|
| Ford      | F150 2015-Present             | Lariat or Higher  | Stock            | 12mph              | 35mph        | 10mph            |
| Ford      | Fusion 2013-Present           | SE or Higher      | Stock            | 12mph              | 35mph        | 10mph            |


# openpilot Capabilities

## Lateral Control

Control over the steering wheel.

Vehicles without LKAS can use openpilot, so long as the LKAS Enable bit is changed in the PSCM with Forscan. This will throw a recurring DTC for a missing IPMA, but will not show up on the IPC. 
Ford currently has a steering lockout on the PSCM, and after 10 seconds, commands will drop for approximately 200-300ms. A fix has not yet been implemented. 

### Torque

### Minimum Speeds
Lateral Control is tied to the Cruise Control. On non Stop/Go vehicles, Lateral stops at 12mph, and can be manually engaged above 20mph. 
On Stop/Go vehicles, Lateral stops at 10mph. 

## Longitudinal Control

Control over the gas and brakes.

Most Ford/Lincoln vehicles do not support OP Longitudinal Control. The CCM on these vehicles (Non Stop/Go) interfaces directly with the HS2 CAN bus and cannot be intercepted. These vehicles run in Lateral Only mode. 

Stop/Go Ford's can be intercepted, but this has not been tested. 

This fork has radar disabled entirely. 

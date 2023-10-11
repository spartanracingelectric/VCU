# XHR-33-18 Power Supply Instructions

## Low Voltage Power Supply Guide for Spartan Racing Electric¬

![DC Power Supply](https://github.com/spartanracingelectric/SRE-2/blob/master/dcpower.jpg)

### How to Turn On  
0.	:warning: **DON’T TURN ON THE POWER SWITCH YET** :warning:   
1.	Press in red :warning: **STANDBY** :warning: button.  Make sure it is clicks in.  
2.	If anything is plugged into the output jacks that you will not be using, unplug that stuff.  
3.	Turn on power supply (big switch on bottom left).  
4.	Hold down V/I button.  Don’t let go until step 7.  
5.	Set voltage  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;a.	Turn voltage knob.  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;b.	Set appropriate value for the circuit you are powering.  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;i.	Car: 12.7 to 14.3 volts  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;ii.	VCU dev board: 12.7 to 14.3 volts  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;iii.	LTC demo board: 7 volts  
6.	Set current limit  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;a.	Turn current knob.  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;b.	Set the value as low as possible for the circuit you are powering.  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;i.	Car: 7 amps (with cooling circuits)  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;ii.	VCU dev board: 2 amps (with relays)  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;iii.	LTC demo board: ???  
7.	Release V/I button.  
8.	Connect the circuit you will be powering  
9.	Turn on power supply  
10.	Unclick the :warning: **STANDBY** :warning: button.  The power outputs will then be enabled.            
              
### Other Settings  
  
*Overvoltage Protection*             
We use overvoltage protection to protect the car in case someone accidentally turns the voltage knob and sets the voltage too high.  It’s set to 15 volts right now, because nothing in the shop needs more than that.  Hold down OVP to adjust.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; I once came into the shop to flash the VCU, and I decided to check the power supply voltage before I started. Good thing &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;I did, because the power supply was set to 28V.  ALWAYS check the voltage before you get started.    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; -Rusty

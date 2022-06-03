Hardware: STM8S105C6
IDE: IAR Embedded Workbench
OS: OSA

Control unit operation algorithm:
The control board have 2 modes of operation:
-	Standby 
- Fire/Alarm
 
 In standby mode, the gate control unit performs standard functions:
- opening, closing, stop (push-button post);
- stop the gate when an obstacle is found during closing (photocells);
- light and sound notification when the gate moves;
- automatic stop of the gate in extreme positions;
- control of power failures (220/ACB);
- readiness to receive a “fire” signal.

In fire/alarm mode:
- the unit switches to the “fire/alarm” mode when a “fire” signal (dry contact) is received;
- the gate immediately begins to close;
- when an obstacle is found by photocells, the gate stops, and after cleaning the obstacles continue to close;
- in this mode, the block should not respond to the open button;
- light and sound notification when the gate moves;
- automatic stop of the gate in extreme positions;
- control of power failures (220 / battery)
- control of the closed position of the gate

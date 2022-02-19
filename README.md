# Rapid React 2022 Robot Code

This is the robot code of the [Spice Gears team 5883](https://www.spicegears.pl) created for the [FRC](https://www.firstinspires.org/robotics/frc/) 2022 competition.

## Installation

In order to properly run the program:

1. Install [WPILib 2022](https://github.com/wpilibsuite/allwpilib/releases/tag/v2022.3.1)
2. Install [NI Tools 2022](https://www.ni.com/pl-pl/support/downloads/drivers/download.frc-game-tools.html)

# Robot design

The robot was designed in Solidworks 2022, you can download it [here](https://grabcad.com/library/spice-gears-5883-frc-robot-1).

### Electronics in robot

Robot has 7 motors

- 6 [CIM](https://www.andymark.com/products/2-5-in-cim-motor) motors _(2 hoist, 4 drive train)_
- 1 [redline](https://www.andymark.com/products/andymark-775-redline-motor-v2) motor _(intake)_

7 controllers:

- 4 [Sparks](https://www.revrobotics.com/rev-11-1200/) _(drive train)_
- 2 [victors SP](https://mindgear.mx/products/victor-sp-speed-controller-am-2855?variant=1188825071643) _(hoist)_
- 1 [victor SP](https://mindgear.mx/products/victor-sp-speed-controller-am-2855?variant=1188825071643) _(intake)_

and others:

- Pneumatic Control Module _[(VEX)](https://www.vexrobotics.com/217-4243.html)_
- Compressor
- 3 Double solenoids _(arm, hook, intake)_
- Voltage Regulator Module _[(VEX)](https://www.vexrobotics.com/217-4245.html)_
- roboRIO _[(NI)](https://www.ni.com/pl-pl/support/model.roborio.html)_
- Power Distribution Panel _[(CTR electronics)](https://store.ctr-electronics.com/power-distribution-panel/#product_tabs_technical_resources)_

### Images

Here you can see the rendered image of the robot, unfortunately we did not have the money to build the whole robot.

_You can see more images **[here](images/)**_

![Robot render image](images/Robot.png)

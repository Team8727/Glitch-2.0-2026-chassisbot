# Glitch 2.0 Programming Documentation

## Getting the robot driving

Check all mechanical parts of the modules (Mk5n type) before attempting to program:

- Check that CANCoder magnets are glued.
- Ensure CANCoders are installed properly.
- Verify that you can control each motor from Phoenix Tuner.
- Notice and address any serious scraping, rattling, or creaking when turning or driving each motor.

## Phoenix Tuner X Connection Tips

- Connect to the robot through the DS Ethernet port in the radio. Make sure the roboRIO is connected to the radio and that the roboRIO has the right firmware version (code issue, see below sections to fix).
- Either enable DriverStation, set team number there and click the DriverStation button in the bottom left dropdown in Phoenix Tuner X, or put in team number (8727) in Phoenix Tuner X. (Try both, then diagnose; the Driver Station method is often fastest and easiest.)
- If plugging into CANivore, check the CANivore USB checkbox and leave the dropdown as `localhost`.

## Calibration and Swerve Code Generation Tips

- Make sure the CAN numbering scheme makes sense and all hardware is labeled and easily identifiable. One easy CAN scheme is numbering each module 10s place by module (10, 20, 30, 40) and the 1s place by function (1 = drive, 2 = steer, 3 = CANCoder).
- Make sure that if required, the Driver Station is open and the robot is enabled.
- Make sure that all devices are on the correct firmware (each device should have a green box in the “Devices” section of Phoenix Tuner X). If a device is yellow, checkbox all the devices needing the firmware and click update in the top right.

## Errors

### Motor(s) are not rotating

- If the CANCoders are flashing red, open the cover and check whether the casing has been installed around the CANCoders. They flash red because they are either too close or too far from the magnet.
- Otherwise, open the Plot for the motor and log the reference position and position. If reference position is changing with either joystick or Tuner X position control, then it is likely a motor constants/PID issue.
- If it’s a turning motor, plot CANCoder position to also check that the CANCoder is not upside-down / magnet is installed and glued down (move a magnetic screwdriver on top of the magnet and check for wobble).
- If the CANCoder is registering position change in Tuner X when you turn the wheel manually but the robot is not driving with the joystick, check the CANCoder installation to make sure they were installed with the housing/magnet/are flashing green. If one of these is wrong (housing missing, no magnet, flashing red), retry after fixing it.

> CANCODERS FLASHING GREEN MEANS THEY ARE OKAY

- If all else fails, factory reset the motor, set an arbitrary P value, and set position. If it moves, try to address what is acting up or consult Chief Delphi for answers.

## Building and Deploying CTRE Swerve Code

- If not done, update all vendor dependencies in the code to the latest version, upgrade WPILib, download the latest NI Gametools image, upgrade RoboRIO version, verify versions of all hardware in CTRE Phoenix Tuner X, and update all hardware to season pass license (by logging in to profile, applying license, adding to each hardware by clicking LIC button, and updating to latest firmware to apply; will display PRO on each motor if licensed properly).
- After running code, if errors with operation occur, watch for indicator lights on hardware (unusual colors/patterns) and read the console.

### Errors

#### Attempting to use Pro License feature on non-licensed device(s)

- Log in to Phoenix Tuner (Profile section) and check whether the right year of license was applied to each motor. Between seasons, it's a common problem to install the next season's license accidentally when the hardware has not been updated/doesn’t have the update yet. To fix, check the license information in the settings, assign the correct license, and then go to devices and add the license to each individually.

#### RoboRIO version is not correct

- Plug into roboRIO using a USB-B cable. This plugs into the single square-ish USB port on the roboRIO, and the standard USB side of the cable plugs into your laptop.
- Load the roboRIO imaging tool, which comes with the standard NI GameTools apps. Find the right roboRIO version and upload it to the roboRIO.

#### RoboRIO memory is too low to write

- Install a USB drive and direct logs and memory to be stored there (have not done this yet).
- Disabling the webserver seemed to provide enough memory to continue temporarily. See this post: RoboRio Not Enough Memory - Technical / Programming - Chief Delphi.

## Tuning the drivetrain

### What is SysID?

Don’t skip this for the drivetrain. It's not super necessary, but it is easy, and it makes the drivetrain sound and drive a bit better (and could be the difference between winning and losing a match).

SysID is a program that calculates the best P, I, and D gains (values tuned to carry out the desired movement of the motor for the mechanism as quickly and accurately as possible). You run SysID to tune the gains for motors on any “arm,” including intakes, multi-DOF mechanisms, and anything that “flips”; “elevator,” including parts that move up and down; or “simple mechanism,” referring to flywheels, swerve motors, and any belts or rollers.

To calculate the optimal PID, SysID allows you to upload logs where the robot (or mechanism other than the drivetrain) follows four rudimentary defined motor routines:

- Quasistatic forward — a constant voltage is applied to the motors in the forward direction.
- Quasistatic reverse.
- Dynamic forward — voltage is ramped up to go forward.
- Dynamic backward.

### SysID guide for finding PID gains for the drivetrain

#### Swerve: Drive Motors

- Run this on the robot (can also be run in simulation to test or practice doing these steps).
- Verify the button triggers in the code to have the drivetrain run each of the SysID routines (automatically in generated swerve code), and set the `m_sysIdRoutineToApply` object in the `CTRESwerveDrivetrain` to the correct type (`m_sysIdRoutineTranslation` for the drive motors, and `m_sysIdRoutineSteer` for the steer motors).

```text
// Inside CTRESwerveDrivetrain: choose which SysID routine to apply
m_sysIdRoutineToApply = m_sysIdRoutineTranslation; // OR m_sysIdRoutineSteer, OR m_sysIdRoutineRotation
```

- Enable the robot, run each of the four SysID routines just once for at least 5–6 seconds each (more is better, as long as the robot doesn’t hit the wall), and don’t otherwise drive or move it in any way.
- Disable the robot, and find and verify the location and existence of the `.hoot` log file for that drive.
- Load the `.hoot` log file found in the logs directory in the code into the Tools section of Tuner X (then the Log Extractor tab), using the interface on the left, and export the log as a WPILOG (Phoenix Tuner X → Tools → Log Extractor → Open './logs/<file>.hoot' → Export → WPILOG). Avoid using the pre-generated WPILOG found in the logs directory in the code.
- Open SysID and load the Tuner-exported WPILOG in. Select the type of calculation in the bottom right panel (Elevator, Arm, Simple). If tuning Swerve use the Simple Mode. Then drag the `SysIDTranslation_State` (or similar name) box into the top of the bottom right panel. For a drive motor, drag the motors `Position`, `Velocity`, and `MotorVoltage` into the three boxes labeled these things. Set the unit to rotations and click load to generate the PID gains (bottom of the center panel).

#### Swerve: Steer Motors

- Run this on the robot (can also be run in simulation to test or practice doing these steps).
- Same as drive, but run the four SysID routines for the steer motors (change the parameters for the triggers in `CTReSwerveControls`).
- Afterward in SysID, drag the `SysIDSteer_State` (or similar name) instead.
- Be sure to change the result (bottom of the center panel) to the loop type “Position” and not velocity.

#### Swerve: Heading PID controller for FieldCentricFaceAngle

- This is the one used in the point-to-hub command for whole-robot rotation tuning.
- Run the SysID routine named this in the `CTRESwerveDrivetrain` for each of the four modes (change the parameters to the triggers for running SysID in `CTReSwerveControls`).
- Follow the details for what to drag into the SysID field at the CTRE API docs for `SwerveRequest.SysIdSwerveRotation`.

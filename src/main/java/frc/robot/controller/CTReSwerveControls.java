package frc.robot.controller;

import Glitch.Lib.NetworkTableLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.Telemetry;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class CTReSwerveControls {

  public CTReSwerveControls(CTRESwerveDrivetrain drivetrain, CommandXboxController controller) {
    double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // PID gains for whole-robot rotation to face a target - different for sim and real (and different from swerve module PID gains)
    double simRotationP = 50;
    double realRotationP = 0.8;

    final NetworkTableLogger netLogger = new NetworkTableLogger("CTReSwerveControls");

    // Swerve Request for normal driving, is the default command
    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // Swerve Request to face a point at (pointX, pointY), not currently used, must place this in a trigger command to use
    final SwerveRequest.FieldCentricFacingAngle facePoint =
      new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    // Swerve Request and target's pose for use in trigger command to always point towards the target.
    Pose3d targetsPose = new Pose3d(
      new Translation3d(10, 4.5, 1.8),
      new Rotation3d());
      // Calculate desired heading in radians
    final SwerveRequest.FieldCentricFacingAngle faceTarget =
      new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.75)
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withHeadingPID(Robot.isReal() ? realRotationP : simRotationP, 0, 0);

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
        drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      ));

    final Telemetry logger = new Telemetry(MaxSpeed);

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    controller.start().and(controller.b()).toggleOnTrue(drivetrain.applyRequest(() -> brake));
    controller.start().and(controller.rightTrigger()).toggleOnTrue(drivetrain.applyRequest(() ->
      point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=- Trigger Command to face a fixed target at (targetX, targetY) -=-=-=-=-=-=-=-=-=-=-=-=-
//    controller.a().whileTrue(drivetrain.applyRequest(() -> {
//      // TEMPORARY!!!: Will need the pose that includes vision measurements added for more accuracy
//      Pose2d robotPose = drivetrain.getState().Pose;
//
//      // Calculate vector and heading from robot to target
//      double dx = targetsPose.getX() - robotPose.getX();
//      double dy = targetsPose.getY() - robotPose.getY();
//
//      // Desired heading calculation
//      double desiredHeadingRadians = Math.atan2(dx, dy);
//
//      // Log targetRobotPose for debugging and testing its accuracy
//      Pose2d targetRobotPose = new Pose2d(
//        robotPose.getTranslation(),
//        new Rotation2d((0.5*Math.PI) - desiredHeadingRadians) // 90-degree offset I had to add for the Pose2d targetRobotPose to point correctly
//      );
//
//      //netLogger.logDouble("desiredHeading: ", desiredHeadingRadians);
//      netLogger.logPose3d("targetsPose: ", targetsPose);
//      netLogger.logPose2d("targetRobotPose: ", targetRobotPose);
//
//      return faceTarget
//        .withTargetDirection(Rotation2d.fromRadians((1.5*Math.PI) - desiredHeadingRadians)) // face the target with 180-degree offset I had to add for some reason
//        .withVelocityX(-controller.getLeftY() * MaxSpeed) // translate across field (driving from red to blue alliance sides)
//        .withVelocityY(-controller.getLeftX() * MaxSpeed); // translate across field (driving from field long wall to other long wall)
//    }));

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=- Trigger Command to point at an angle to hit a target with a projectile using ProjectileSolver -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    controller.a().whileTrue(drivetrain.applyRequest(() -> {
      return faceTarget
              .withTargetDirection(Rotation2d.fromDegrees(Robot.firing.yaw)) // face the target with 180-degree offset I had to add for some reason
              .withVelocityX(-controller.getLeftY() * MaxSpeed) // translate across field (driving from red to blue alliance sides)
              .withVelocityY(-controller.getLeftX() * MaxSpeed); // translate across field (driving from field long wall to other long wall)
    }));
// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    controller.povUp().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward)); // 2
    controller.povLeft().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse)); // 1
    controller.povDown().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward)); // 4
    controller.povRight().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)); // 3

    // reset the field-centric heading on left bumper press    // reset the field-centric heading on left bumper press
    controller.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
  }
}

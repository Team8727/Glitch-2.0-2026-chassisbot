package frc.robot.controller;

import Glitch.Lib.NetworkTableLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Drivetrain.Telemetry;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class CTReSwerveControls {

  // PID gains for whole-robot rotation to face a target - different for sim and real (and different from swerve module PID gains)
  static final double SIM_ROTATION_kP = 50;
  static final double REAL_ROTATION_kP = 0.8;

  // Hub positions
//  private static final Translation3d BLUE_ALLIANCE_TARGET_3D = new Translation3d(4.626, 4.035, 1.8);
//  private static final Translation3d RED_ALLIANCE_TARGET_3D = new Translation3d(11.915, 4.035, 1.8);

  // Will be one of the hub positions depending on alliance color. See in point to hub trigger command below.
//  private Translation3d target;

  public CTReSwerveControls(CTRESwerveDrivetrain drivetrain, CommandXboxController controller) {
    double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withHeadingPID(Robot.isReal() ? REAL_ROTATION_kP : SIM_ROTATION_kP, 0, 0);

    // Swerve Request for use in trigger command to always point towards the target.
    final SwerveRequest.FieldCentricFacingAngle faceTarget =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.75)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withHeadingPID(Robot.isReal() ? REAL_ROTATION_kP : SIM_ROTATION_kP, 0, 0);

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                    drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                            .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                            .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    final Telemetry logger = new Telemetry(MaxSpeed);

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=- Trigger Commands on disable, to automatically brake when stopped, and to point wheels with left stick on pressing certain buttons -=-=-=-=-=-=-=-=-=-=-=-=-
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Automatically brake (and put wheels in X) when the robot is stopped (within deadband)
    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    new Trigger(() -> Math.abs(controller.getLeftY()) < 0.1 && Math.abs(controller.getLeftX()) < 0.1 && Math.abs(controller.getRightX()) < 0.1 && controller.a().negate().getAsBoolean())
            .whileTrue(drivetrain.applyRequest(() -> brake));

    // Point wheels in direction of left stick when pressing right trigger and start button together
    final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    controller.start().and(controller.rightTrigger()).toggleOnTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=- Trigger Command to face a fixed target at (targetX, targetY) -=-=-=-=-=-=-=-=-=-=-=-=-
//    controller.a().whileTrue(drivetrain.applyRequest(() -> {
//      target = Robot.isRedAlliance() ? RED_ALLIANCE_TARGET_3D : BLUE_ALLIANCE_TARGET_3D;
//
//      // TEMPORARY!!!: Will need the pose that includes vision measurements added for more accuracy
//      Pose2d robotPose = drivetrain.getState().Pose;
//
//      // Calculate vector and heading from robot to target
//      double dx = target.getX() - robotPose.getX();
//      double dy = target.getY() - robotPose.getY();
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
//      netLogger.logPose3d("target: ", new Pose3d(target, new Rotation3d()));
//      netLogger.logPose2d("targetRobotPose: ", targetRobotPose);
//
//      return faceTarget
//        .withTargetDirection(Rotation2d.fromRadians((1.5*Math.PI) - desiredHeadingRadians)) // face the target with 180-degree offset I had to add for some reason
//        .withVelocityX(-controller.getLeftY() * MaxSpeed) // translate across field (driving from red to blue alliance sides)
//        .withVelocityY(-controller.getLeftX() * MaxSpeed); // translate across field (driving from field long wall to other long wall)
//    }));

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=- Trigger Command to point at an angle to hit a target with a projectile using ProjectileSolver -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    controller.a().whileTrue(drivetrain.applyRequest(() -> {
      return faceTarget
              .withTargetDirection(Rotation2d.fromDegrees(Robot.firing.yaw)) // face the target with 180-degree offset I had to add for some reason
              .withVelocityX(-controller.getLeftY() * MaxSpeed) // translate across field (driving from red to blue alliance sides)
              .withVelocityY(-controller.getLeftX() * MaxSpeed); // translate across field (driving from field long wall to other long wall)
    }));

// -=-=-=-=-=-=-=-=-=-=-=-=-=-= SysID characterization for driving and turning (but not heading controller, unless you add a trigger for that) -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
//    controller.povUp().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward)); // 2
//    controller.povLeft().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse)); // 1
//    controller.povDown().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward)); // 4
//    controller.povRight().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)); // 3

// -=-=-=-=-=-=-=-=-=-=-=-=-=- Reset field-centric heading -=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // reset the field-centric heading on left bumper press    // reset the field-centric heading on left bumper press
    controller.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=- Telemetry registration -=-=-=-=-=-=-=-=-=-=-=-=-
    drivetrain.registerTelemetry(logger::telemeterize);
  }
}

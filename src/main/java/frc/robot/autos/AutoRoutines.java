package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * Autonomous routines for 2026 REBUILT - THOR West.
 *
 * <p>Each routine comes in 3 starting positions (J-Left, J-Center, J-Right).
 * Uses PathPlanner online pathfinding. All poses in WPILib center-origin frame
 * (blue convention; mirrored for red via {@link #alliancePose(Pose2d)}).
 *
 * <h3>Starting Positions</h3>
 * <ul>
 *   <li><b>J-Left</b> — starting from the left side of the alliance wall
 *       (depot side). Optimal for depot rush.
 *   <li><b>J-Center</b> — starting from the center of the alliance wall
 *       (between the bumps). Best for center sweep.
 *   <li><b>J-Right</b> — starting from the right side of the alliance wall
 *       (outpost side). Good for outpost intake.
 * </ul>
 */
public class AutoRoutines {
  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private final Intake intake;
  private final Indexer indexer;

  private static final PathConstraints kConstraints = new PathConstraints(
      Constants.kAutoMaxLinearVelocity,
      Constants.kAutoMaxLinearAcceleration,
      Constants.kAutoMaxAngularVelocity,
      Constants.kAutoMaxAngularAcceleration);

  // ========================================================================
  // Field Positions (blue-alliance frame, center-origin)
  //
  // WPILib center-origin: origin at center of field, +X toward RED alliance.
  // Blue wall is at X = -8.27, Red wall at X = +8.27.
  // Y = 0 is the centerline; +Y is to the left (from blue driver perspective).
  // ========================================================================

  // ---- Shoot positions ----
  /** Shoot from left side: about 2m in front of HUB, slightly left. */
  private static final Pose2d kBlueShootLeft =
      new Pose2d(-5.8, 1.8, Rotation2d.fromDegrees(180));

  /** Shoot from center: directly in front of HUB. */
  private static final Pose2d kBlueShootCenter =
      new Pose2d(-5.8, 0.0, Rotation2d.fromDegrees(180));

  /** Shoot from right side: about 2m in front of HUB, slightly right. */
  private static final Pose2d kBlueShootRight =
      new Pose2d(-5.8, -1.8, Rotation2d.fromDegrees(180));

  // ---- Intake positions ----
  /** Depot approach (left side of alliance wall). */
  private static final Pose2d kBlueDepotPose =
      new Pose2d(-7.8, 2.5, Rotation2d.fromDegrees(180));

  /** Outpost approach (right side of alliance wall). */
  private static final Pose2d kBlueOutpostPose =
      new Pose2d(-7.8, -2.5, Rotation2d.fromDegrees(180));

  /** Center-field sweep end — deep in neutral zone, center. */
  private static final Pose2d kBlueSweepCenterEnd =
      new Pose2d(1.5, 0.0, Rotation2d.fromDegrees(180));

  /** Center-field sweep end — deep in neutral zone, left side. */
  private static final Pose2d kBlueSweepLeftEnd =
      new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(180));

  /** Center-field sweep end — deep in neutral zone, right side. */
  private static final Pose2d kBlueSweepRightEnd =
      new Pose2d(1.5, -1.5, Rotation2d.fromDegrees(180));

  // ---- Mobility poses ----
  private static final Pose2d kBlueMobilityCenter =
      new Pose2d(-1.0, 0.0, Rotation2d.fromDegrees(0));

  private static final Pose2d kBlueMobilityLeft =
      new Pose2d(-1.0, 1.8, Rotation2d.fromDegrees(0));

  private static final Pose2d kBlueMobilityRight =
      new Pose2d(-1.0, -1.8, Rotation2d.fromDegrees(0));

  // ========================================================================
  // Construction
  // ========================================================================

  public AutoRoutines(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.indexer = indexer;
    registerNamedCommands();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("intake",
        intake.run(() -> intake.setDutyCycle(Constants.kIntakeDutyCycle))
            .alongWith(indexer.run(
                () -> indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle)))
            .finallyDo(() -> {
              intake.setDutyCycle(0);
              indexer.setDutyCycle(0);
            }));

    NamedCommands.registerCommand("stopIntake",
        runOnce(() -> {
          intake.setDutyCycle(0);
          indexer.setDutyCycle(0);
        }));

    NamedCommands.registerCommand("shoot",
        new ShootCommand(shooter, indexer, drivetrain));

    NamedCommands.registerCommand("stopShooter",
        runOnce(() -> shooter.setDutyCycle(0)));
  }

  // ========================================================================
  // Alliance Mirroring
  // ========================================================================

  private static Pose2d mirrorPose(Pose2d bluePose) {
    return new Pose2d(
        -bluePose.getX(),
        bluePose.getY(),
        bluePose.getRotation().rotateBy(Rotation2d.k180deg));
  }

  /** Returns the same physical location for the current alliance. */
  private Pose2d alliancePose(Pose2d bluePose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return mirrorPose(bluePose);
    }
    return bluePose;
  }

  // ========================================================================
  // Building Blocks
  // ========================================================================

  private Command shootPreloads() {
    return new ShootCommand(shooter, indexer, drivetrain);
  }

  private Command pathfindTo(Pose2d targetPose) {
    return AutoBuilder.pathfindToPose(targetPose, kConstraints, 0.0);
  }

  /** Runs intake + indexer for a set duration, then stops. */
  private Command runIntake(double durationSeconds) {
    return sequence(
        runOnce(() -> {
          intake.setDutyCycle(Constants.kIntakeDutyCycle);
          indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
        }),
        waitSeconds(durationSeconds),
        runOnce(() -> {
          intake.setDutyCycle(0);
          indexer.setDutyCycle(0);
        }));
  }

  /** Intake while pathfinding, then continue running intake for more time. */
  private Command intakeWhileMoving(Pose2d target, double intakeAfterSec) {
    return sequence(
        parallel(
            pathfindTo(alliancePose(target)),
            sequence(
                waitSeconds(3.0),
                runOnce(() -> {
                  intake.setDutyCycle(Constants.kIntakeDutyCycle);
                  indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
                }))),
        runIntake(intakeAfterSec));
  }

  // ========================================================================
  // Auto Routines — Each has Left / Center / Right variants
  // ========================================================================

  // -------- 2-Cycle Depot Rush (J-Left | J-Center | J-Right) --------

  /**
   * 2-Cycle Depot Rush — J-Left.
   * Shoot from left side, intake from left-side depot, return to shoot.
   */
  public Command twoCycleDepotRushLeft() {
    return sequence(
        shootPreloads(),
        intakeWhileMoving(kBlueDepotPose, 3.0),
        pathfindTo(alliancePose(kBlueShootLeft)),
        shootPreloads());
  }

  /**
   * 2-Cycle Depot Rush — J-Center.
   * Shoot from center, intake from depot (left side), return to center to shoot.
   */
  public Command twoCycleDepotRushCenter() {
    return sequence(
        shootPreloads(),
        intakeWhileMoving(kBlueDepotPose, 3.0),
        pathfindTo(alliancePose(kBlueShootCenter)),
        shootPreloads());
  }

  /**
   * 2-Cycle Depot Rush — J-Right.
   * Shoot from right side, intake from outpost (right side), return to shoot.
   */
  public Command twoCycleDepotRushRight() {
    return sequence(
        shootPreloads(),
        intakeWhileMoving(kBlueOutpostPose, 3.0),
        pathfindTo(alliancePose(kBlueShootRight)),
        shootPreloads());
  }

  // -------- Center Sweep (J-Left | J-Center | J-Right) --------

  /**
   * Center Sweep — J-Left.
   * Shoot from left, sweep left side of neutral zone, return to shoot.
   */
  public Command centerSweepLeft() {
    return sequence(
        shootPreloads(),
        parallel(
            pathfindTo(alliancePose(kBlueSweepLeftEnd)),
            sequence(
                runOnce(() -> {
                  intake.setDutyCycle(Constants.kIntakeDutyCycle);
                  indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
                }),
                waitSeconds(3.5),
                runOnce(() -> {
                  intake.setDutyCycle(0);
                  indexer.setDutyCycle(0);
                }))),
        pathfindTo(alliancePose(kBlueShootLeft)),
        shootPreloads());
  }

  /**
   * Center Sweep — J-Center.
   * Shoot from center, sweep center of neutral zone, return to shoot.
   */
  public Command centerSweepCenter() {
    return sequence(
        shootPreloads(),
        parallel(
            pathfindTo(alliancePose(kBlueSweepCenterEnd)),
            sequence(
                runOnce(() -> {
                  intake.setDutyCycle(Constants.kIntakeDutyCycle);
                  indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
                }),
                waitSeconds(3.5),
                runOnce(() -> {
                  intake.setDutyCycle(0);
                  indexer.setDutyCycle(0);
                }))),
        pathfindTo(alliancePose(kBlueShootCenter)),
        shootPreloads());
  }

  /**
   * Center Sweep — J-Right.
   * Shoot from right, sweep right side of neutral zone, return to shoot.
   */
  public Command centerSweepRight() {
    return sequence(
        shootPreloads(),
        parallel(
            pathfindTo(alliancePose(kBlueSweepRightEnd)),
            sequence(
                runOnce(() -> {
                  intake.setDutyCycle(Constants.kIntakeDutyCycle);
                  indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
                }),
                waitSeconds(3.5),
                runOnce(() -> {
                  intake.setDutyCycle(0);
                  indexer.setDutyCycle(0);
                }))),
        pathfindTo(alliancePose(kBlueShootRight)),
        shootPreloads());
  }

  // -------- Shoot + Mobility (J-Left | J-Center | J-Right) --------

  public Command shootAndLeaveLeft() {
    return sequence(
        shootPreloads(),
        pathfindTo(alliancePose(kBlueMobilityLeft)));
  }

  public Command shootAndLeaveCenter() {
    return sequence(
        shootPreloads(),
        pathfindTo(alliancePose(kBlueMobilityCenter)));
  }

  public Command shootAndLeaveRight() {
    return sequence(
        shootPreloads(),
        pathfindTo(alliancePose(kBlueMobilityRight)));
  }

  // -------- Shoot In Place (J-Left | J-Center | J-Right) --------

  public Command shootInPlaceLeft() {
    return shootPreloads();
  }

  public Command shootInPlaceCenter() {
    return shootPreloads();
  }

  public Command shootInPlaceRight() {
    return shootPreloads();
  }
}
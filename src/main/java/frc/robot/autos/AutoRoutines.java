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
 * Autonomous routines for 2026 REBUILT.
 *
 * <p>Each routine composes shooting, intake, and PathPlanner pathfinding
 * commands. Uses online pathfinding instead of .path files for robustness
 * during the THOR West off-season.
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

  private static final PathConstraints kSlowConstraints = new PathConstraints(
      Constants.kAutoMaxLinearVelocity * 0.5,
      Constants.kAutoMaxLinearAcceleration * 0.6,
      Constants.kAutoMaxAngularVelocity * 0.5,
      Constants.kAutoMaxAngularAcceleration * 0.5);

  // ========================================================================
  // Field Positions (blue-alliance frame, center-origin)
  // ========================================================================

  /** Primary shooting position: ~2m in front of blue HUB. */
  private static final Pose2d kBlueShootPose =
      new Pose2d(-5.8, 0.0, Rotation2d.fromDegrees(180));

  /** Approach pose for the depot area (near alliance wall). */
  private static final Pose2d kBlueDepotPose =
      new Pose2d(-7.8, -2.5, Rotation2d.fromDegrees(180));

  /** Center-field sweep end - deep in neutral zone. */
  private static final Pose2d kBlueSweepEnd =
      new Pose2d(1.5, 0.0, Rotation2d.fromDegrees(180));

  /** Mobility pose - across the auto line. */
  private static final Pose2d kBlueMobilityPose =
      new Pose2d(-2.0, 0.0, Rotation2d.fromDegrees(0));

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

  // ========================================================================
  // Auto Routines
  // ========================================================================

  /**
   * AUTO 1: "2-Cycle Depot Rush" (DEFAULT)
   *
   * Shoot preloads, drive to alliance depot, intake FUEL, return to shoot.
   * Phases: (0-2s) shoot 8 preloads, (2-6s) pathfind to depot, (6-10s) intake,
   * (10-14s) pathfind back, (14-20s) shoot collected FUEL.
   */
  public Command twoCycleDepotRush() {
    return sequence(
        shootPreloads(),
        parallel(
            pathfindTo(alliancePose(kBlueDepotPose)),
            sequence(
                waitSeconds(3.5),
                runOnce(() -> {
                  intake.setDutyCycle(Constants.kIntakeDutyCycle);
                  indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
                }))),
        runIntake(4.0),
        pathfindTo(alliancePose(kBlueShootPose)),
        shootPreloads());
  }

  /**
   * AUTO 2: "Center Sweep"
   *
   * Shoot preloads, drive through neutral zone collecting FUEL, return
   * to shoot. More reliable but fewer balls than depot rush.
   */
  public Command centerSweep() {
    return sequence(
        shootPreloads(),
        parallel(
            pathfindTo(alliancePose(kBlueSweepEnd)),
            sequence(
                runOnce(() -> {
                  intake.setDutyCycle(Constants.kIntakeDutyCycle);
                  indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
                }),
                waitSeconds(3.0),
                runOnce(() -> {
                  intake.setDutyCycle(0);
                  indexer.setDutyCycle(0);
                }))),
        pathfindTo(alliancePose(kBlueShootPose)),
        shootPreloads());
  }

  /**
   * AUTO 3: "Shoot + Mobility"
   *
   * Shoot preloads and drive across the auto line for mobility points.
   * Simple and reliable.
   */
  public Command shootAndLeave() {
    return sequence(
        shootPreloads(),
        pathfindTo(alliancePose(kBlueMobilityPose)));
  }

  /**
   * AUTO 4: "Shoot In Place"
   *
   * Just fire preloads from starting position. 100% reliable.
   */
  public Command shootInPlace() {
    return shootPreloads();
  }
}
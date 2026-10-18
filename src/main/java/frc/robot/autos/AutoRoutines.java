package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.json.simple.parser.ParseException;

import java.io.IOException;

import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * Defines autonomous routines for the 2026 REBUILT game.
 *
 * <p>Each routine is composed of PathPlanner paths and shooting/intake commands.
 * Paths are loaded from the deploy/pathplanner/paths/ directory.
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

  public AutoRoutines(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.indexer = indexer;

    registerNamedCommands();
  }

  /** Registers commands that can be triggered by PathPlanner event markers. */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("intake",
        intake.run(() -> intake.setDutyCycle(Constants.kIntakeDutyCycle))
            .alongWith(indexer.run(() -> indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle)))
            .finallyDo(() -> {
              intake.setDutyCycle(0);
              indexer.setDutyCycle(0);
            }));

    NamedCommands.registerCommand("shoot", new ShootCommand(shooter, indexer, drivetrain));

    NamedCommands.registerCommand("stopShooter", runOnce(() -> shooter.setDutyCycle(0)));
  }

  /**
   * Load and follow a path, optionally mirroring for the red alliance.
   *
   * @param name   Path file name (without .path extension)
   * @param mirror Whether to mirror the path for the current alliance
   */
  private Command followPath(String name, boolean mirror) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(name);
      if (mirror && DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        path = path.flipPath();
      }
      Pose2d startPose = path.getStartingHolonomicPose().orElseThrow();
      return sequence(
          runOnce(() -> drivetrain.resetPose(startPose)),
          AutoBuilder.followPath(path));
    } catch (IOException | ParseException e) {
      DriverStation.reportError("Failed to load path: " + name + " (" + e.getMessage() + ")", true);
      return none();
    }
  }

  // ========================================================================
  // Auto Routine Definitions
  // ========================================================================

  /**
   * 4-Piece: Shoot preloaded → drive to depot → intake → return → shoot → leave.
   * This is the highest-scoring auto and the default selection.
   */
  public Command fourPieceAuto() {
    return sequence(
        followPath("Final plan 4.1", true),
        new ShootCommand(shooter, indexer, drivetrain),
        followPath("Depot-PreDepot", true),
        intake.run(() -> intake.setDutyCycle(Constants.kIntakeDutyCycle))
            .withTimeout(2.0),
        followPath("PreDepot-Depot", true),
        new ShootCommand(shooter, indexer, drivetrain));
  }

  /**
   * 2-Piece + Leave: Shoot preloaded → intake midline → shoot → drive across line.
   */
  public Command twoPieceAuto() {
    return sequence(
        followPath("Close Half Middle", true),
        new ShootCommand(shooter, indexer, drivetrain),
        followPath("NeutralBumpRightWall-NeutralBumpRightMid", true),
        new ShootCommand(shooter, indexer, drivetrain));
  }

  /** Shoot preloads and drive across the line for mobility points. */
  public Command shootAndLeave() {
    return sequence(
        new ShootCommand(shooter, indexer, drivetrain),
        followPath("Far Half Middle", true));
  }

  /** Shoot preloads in place (no movement). */
  public Command shootInPlace() {
    return new ShootCommand(shooter, indexer, drivetrain);
  }

  /** Simple 2-piece: shoot, drive to nearest pickup, return, shoot. */
  public Command simpleTwoPiece() {
    return sequence(
        new ShootCommand(shooter, indexer, drivetrain),
        followPath("test", true),
        new ShootCommand(shooter, indexer, drivetrain));
  }
}
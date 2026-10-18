package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
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
  // Blue-alliance positions (center-origin WPILib frame)
  // ========================================================================

  private static final Pose2d kBlueShootLeft    = new Pose2d(-5.8, 2.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueShootCenter  = new Pose2d(-5.8, 0.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueShootRight   = new Pose2d(-5.8, -2.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueDepot        = new Pose2d(-7.8, 2.5, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueOutpost      = new Pose2d(-7.8, -2.5, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueSweepLeft    = new Pose2d(1.5, 2.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueSweepCenter  = new Pose2d(1.5, 0.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueSweepRight   = new Pose2d(1.5, -2.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueMobilityLeft    = new Pose2d(-1.0, 2.0, Rotation2d.fromDegrees(0));
  private static final Pose2d kBlueMobilityCenter  = new Pose2d(-1.0, 0.0, Rotation2d.fromDegrees(0));
  private static final Pose2d kBlueMobilityRight   = new Pose2d(-1.0, -2.0, Rotation2d.fromDegrees(0));

  // PathPlanner path file names (blue-corner-origin, in deploy/pathplanner/paths/)
  private static final String kPathLeftToShoot     = "J-Left to Shoot";
  private static final String kPathShootToDepot    = "Shoot to Depot Left";
  private static final String kPathDepotToShoot    = "Depot Left to Shoot";
  private static final String kPathCenterToShoot   = "J-Center to Shoot";
  private static final String kPathShootToSweep    = "Shoot to Sweep Center";
  private static final String kPathSweepToShoot    = "Sweep Center to Shoot";
  private static final String kPathRightToShoot    = "J-Right to Shoot";
  private static final String kPathShootToOutpost  = "Shoot to Outpost Right";
  private static final String kPathOutpostToShoot  = "Outpost Right to Shoot";
  private static final String kPathShootToMobility = "Shoot to Mobility";

  // ========================================================================

  public AutoRoutines(Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.drivetrain = drivetrain; this.shooter = shooter; this.intake = intake; this.indexer = indexer;
    registerNamedCommands();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("intake",
        intake.run(() -> intake.setDutyCycle(Constants.kIntakeDutyCycle))
            .alongWith(indexer.run(() -> indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle)))
            .finallyDo(() -> { intake.setDutyCycle(0); indexer.setDutyCycle(0); }));
    NamedCommands.registerCommand("stopIntake",
        runOnce(() -> { intake.setDutyCycle(0); indexer.setDutyCycle(0); }));
    NamedCommands.registerCommand("shoot", new ShootCommand(shooter, indexer, drivetrain));
    NamedCommands.registerCommand("stopShooter", runOnce(() -> shooter.setDutyCycle(0)));
  }

  // ========================================================================
  // Mirroring
  // ========================================================================

  private static Pose2d mirror(Pose2d p) {
    return new Pose2d(-p.getX(), p.getY(), p.getRotation().rotateBy(Rotation2d.k180deg));
  }
  private Pose2d ap(Pose2d bluePose) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? mirror(bluePose) : bluePose;
  }

  // ========================================================================
  // Building blocks
  // ========================================================================

  private Command shoot() { return new ShootCommand(shooter, indexer, drivetrain); }

  private Command pathfind(Pose2d p) {
    return AutoBuilder.pathfindToPose(ap(p), kConstraints, 0.0);
  }

  /** Try .path file first, fall back to pathfinding. */
  private Command go(String pathName, Pose2d fallback) {
    try {
      var path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      return pathfind(fallback);
    }
  }

  /** Path with red-alliance mirroring. */
  private Command goMirrored(String pathName, Pose2d fallback) {
    try {
      var path = PathPlannerPath.fromPathFile(pathName);
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        return AutoBuilder.followPath(path.flipPath());
      }
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      return pathfind(fallback);
    }
  }

  private Command intakeWhileMoving(String pathName, Pose2d target, Pose2d shootPos) {
    return sequence(
        parallel(
            go(pathName, target),
            sequence(waitSeconds(3.0), runOnce(() -> {
              intake.setDutyCycle(Constants.kIntakeDutyCycle);
              indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
            }))),
        run(() -> {}) // placeholder — intake continues
    );
  }

  // ========================================================================
  // Auto Routines
  // ========================================================================

  // --- 2-Cycle Depot Rush ---

  public Command twoCycleDepotRushLeft() {
    return sequence(
        shoot(),
        go(kPathShootToDepot, kBlueDepot),
        runIntakeSec(3.0),
        go(kPathDepotToShoot, kBlueShootLeft),
        shoot());
  }

  public Command twoCycleDepotRushCenter() {
    return sequence(
        shoot(),
        go(kPathShootToDepot, kBlueDepot),
        runIntakeSec(3.0),
        go(kPathSweepToShoot, kBlueShootCenter),
        shoot());
  }

  public Command twoCycleDepotRushRight() {
    return sequence(
        shoot(),
        go(kPathShootToOutpost, kBlueOutpost),
        runIntakeSec(3.0),
        go(kPathOutpostToShoot, kBlueShootRight),
        shoot());
  }

  // --- Center Sweep ---

  public Command centerSweepLeft() {
    return sequence(
        shoot(),
        go(kPathShootToSweep, kBlueSweepLeft),
        runIntakeSec(2.0),
        go(kPathSweepToShoot, kBlueShootLeft),
        shoot());
  }

  public Command centerSweepCenter() {
    return sequence(
        shoot(),
        go(kPathShootToSweep, kBlueSweepCenter),
        runIntakeSec(2.0),
        go(kPathSweepToShoot, kBlueShootCenter),
        shoot());
  }

  public Command centerSweepRight() {
    return sequence(
        shoot(),
        go(kPathShootToSweep, kBlueSweepRight),
        runIntakeSec(2.0),
        go(kPathSweepToShoot, kBlueShootRight),
        shoot());
  }

  // --- Shoot + Mobility ---

  public Command shootAndLeaveLeft() {
    return sequence(shoot(), go(kPathShootToMobility, kBlueMobilityLeft));
  }
  public Command shootAndLeaveCenter() {
    return sequence(shoot(), go(kPathShootToMobility, kBlueMobilityCenter));
  }
  public Command shootAndLeaveRight() {
    return sequence(shoot(), go(kPathShootToMobility, kBlueMobilityRight));
  }

  // --- Shoot In Place ---

  public Command shootInPlaceLeft()   { return shoot(); }
  public Command shootInPlaceCenter() { return shoot(); }
  public Command shootInPlaceRight()  { return shoot(); }

  // ========================================================================
  // Helper
  // ========================================================================

  private Command runIntakeSec(double seconds) {
    return sequence(
        runOnce(() -> {
          intake.setDutyCycle(Constants.kIntakeDutyCycle);
          indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
        }),
        waitSeconds(seconds),
        runOnce(() -> { intake.setDutyCycle(0); indexer.setDutyCycle(0); }));
  }
}
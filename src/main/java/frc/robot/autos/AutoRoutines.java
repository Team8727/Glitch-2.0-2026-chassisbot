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
  // Blue-alliance positions (WPILib center-origin frame)
  // ========================================================================

  // Starting positions (must match .path file start waypoints)
  // Converted from PathPlanner frame: PP_X - 8.27, PP_Y - 4.035
  // J-Left to Shoot starts at PP(1.8, 6.0)   -> WPILib(-6.47, 1.97)
  // J-Center to Shoot starts at PP(1.8, 4.035) -> WPILib(-6.47, 0.0)
  // J-Right to Shoot starts at PP(1.8, 2.0)   -> WPILib(-6.47, -2.04)
  private static final Pose2d kBlueStartLeft    = new Pose2d(-6.5,  2.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueStartCenter  = new Pose2d(-6.5,  0.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueStartRight   = new Pose2d(-6.5, -2.0, Rotation2d.fromDegrees(180));

  // Shoot positions
  private static final Pose2d kBlueShootLeft    = new Pose2d(-5.8,  2.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueShootCenter  = new Pose2d(-5.8,  0.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueShootRight   = new Pose2d(-5.8, -2.0, Rotation2d.fromDegrees(180));

  // Intake positions
  private static final Pose2d kBlueDepot        = new Pose2d(-7.8,  2.5, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueOutpost      = new Pose2d(-7.8, -2.5, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueSweepLeft    = new Pose2d( 1.5,  2.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueSweepCenter  = new Pose2d( 1.5,  0.0, Rotation2d.fromDegrees(180));
  private static final Pose2d kBlueSweepRight   = new Pose2d( 1.5, -2.0, Rotation2d.fromDegrees(180));

  // Mobility positions
  private static final Pose2d kBlueMobilityLeft    = new Pose2d(-1.0,  2.0, Rotation2d.fromDegrees(0));
  private static final Pose2d kBlueMobilityCenter  = new Pose2d(-1.0,  0.0, Rotation2d.fromDegrees(0));
  private static final Pose2d kBlueMobilityRight   = new Pose2d(-1.0, -2.0, Rotation2d.fromDegrees(0));

  // Path file names
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

  public AutoRoutines(Drivetrain dt, Shooter s, Intake i, Indexer ix) {
    drivetrain = dt; shooter = s; intake = i; indexer = ix;
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
  // Helpers
  // ========================================================================

  private static Pose2d mirror(Pose2d p) {
    return new Pose2d(-p.getX(), p.getY(), p.getRotation().rotateBy(Rotation2d.k180deg));
  }
  private Pose2d ap(Pose2d bluePose) {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? mirror(bluePose) : bluePose;
  }

  /** Reset drivetrain pose so the robot knows where it is (critical without vision). */
  private Command resetPose(Pose2d startPose) {
    return runOnce(() -> drivetrain.resetPose(ap(startPose)));
  }

  private Command shoot() { return new ShootCommand(shooter, indexer, drivetrain); }

  private Command pathfind(Pose2d p) {
    return AutoBuilder.pathfindToPose(ap(p), kConstraints, 0.0);
  }

  /** Try .path file first, fall back to online pathfinding. */
  private Command go(String pathName, Pose2d fallback) {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    } catch (Exception e) {
      return pathfind(fallback);
    }
  }

  private Command runIntakeSec(double seconds) {
    return sequence(
        runOnce(() -> { intake.setDutyCycle(Constants.kIntakeDutyCycle);
                        indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle); }),
        waitSeconds(seconds),
        runOnce(() -> { intake.setDutyCycle(0); indexer.setDutyCycle(0); }));
  }

  // ========================================================================
  // Auto Routines — each resets pose first so autos work without vision
  // ========================================================================

  // --- 2-Cycle Depot Rush ---

  public Command twoCycleDepotRushLeft() {
    return sequence(resetPose(kBlueStartLeft),
        shoot(),
        go(kPathShootToDepot, kBlueDepot), runIntakeSec(3.0),
        go(kPathDepotToShoot, kBlueShootLeft), shoot());
  }

  public Command twoCycleDepotRushCenter() {
    return sequence(resetPose(kBlueStartCenter),
        shoot(),
        go(kPathShootToDepot, kBlueDepot), runIntakeSec(3.0),
        go(kPathSweepToShoot, kBlueShootCenter), shoot());
  }

  public Command twoCycleDepotRushRight() {
    return sequence(resetPose(kBlueStartRight),
        shoot(),
        go(kPathShootToOutpost, kBlueOutpost), runIntakeSec(3.0),
        go(kPathOutpostToShoot, kBlueShootRight), shoot());
  }

  // --- Center Sweep ---

  public Command centerSweepLeft() {
    return sequence(resetPose(kBlueStartLeft),
        shoot(),
        go(kPathShootToSweep, kBlueSweepLeft), runIntakeSec(2.0),
        go(kPathSweepToShoot, kBlueShootLeft), shoot());
  }

  public Command centerSweepCenter() {
    return sequence(resetPose(kBlueStartCenter),
        shoot(),
        go(kPathShootToSweep, kBlueSweepCenter), runIntakeSec(2.0),
        go(kPathSweepToShoot, kBlueShootCenter), shoot());
  }

  public Command centerSweepRight() {
    return sequence(resetPose(kBlueStartRight),
        shoot(),
        go(kPathShootToSweep, kBlueSweepRight), runIntakeSec(2.0),
        go(kPathSweepToShoot, kBlueShootRight), shoot());
  }

  // --- Shoot + Mobility ---

  public Command shootAndLeaveLeft() {
    return sequence(resetPose(kBlueStartLeft), shoot(), go(kPathShootToMobility, kBlueMobilityLeft));
  }
  public Command shootAndLeaveCenter() {
    return sequence(resetPose(kBlueStartCenter), shoot(), go(kPathShootToMobility, kBlueMobilityCenter));
  }
  public Command shootAndLeaveRight() {
    return sequence(resetPose(kBlueStartRight), shoot(), go(kPathShootToMobility, kBlueMobilityRight));
  }

  // --- Shoot In Place ---

  public Command shootInPlaceLeft()   { return sequence(resetPose(kBlueStartLeft), shoot()); }
  public Command shootInPlaceCenter() { return sequence(resetPose(kBlueStartCenter), shoot()); }
  public Command shootInPlaceRight()  { return sequence(resetPose(kBlueStartRight), shoot()); }
}
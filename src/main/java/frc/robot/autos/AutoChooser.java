package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Dashboard auto mode selector.
 *
 * <p>Exposes a {@link SendableChooser} on SmartDashboard that lets the driver
 * select which autonomous routine to run. All routines are designed for the
 * 2026 REBUILT game at THOR West.
 *
 * <h3>Available Routines</h3>
 * <ol>
 *   <li><b>2-Cycle Depot Rush</b> (default) — shoot preloads, intake from
 *       depot, return to shoot. Highest scoring potential.
 *   <li><b>Center Sweep</b> — shoot, sweep neutral zone, return to shoot.
 *       More reliable but fewer balls.
 *   <li><b>Shoot + Mobility</b> — shoot preloads and drive across the line.
 *       Simple and reliable.
 *   <li><b>Shoot In Place</b> — just fire preloads from starting position.
 *       100% reliable.
 * </ol>
 */
public class AutoChooser {
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  private final AutoRoutines routines;

  public AutoChooser(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.routines = new AutoRoutines(drivetrain, shooter, intake, indexer);

    chooser.setDefaultOption("2-Cycle Depot Rush", routines.twoCycleDepotRush());
    chooser.addOption("Center Sweep", routines.centerSweep());
    chooser.addOption("Shoot + Mobility", routines.shootAndLeave());
    chooser.addOption("Shoot In Place", routines.shootInPlace());

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  /** @return The currently selected autonomous command. */
  public Command getSelected() {
    return chooser.getSelected();
  }
}
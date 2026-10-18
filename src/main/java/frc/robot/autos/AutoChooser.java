package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Dashboard auto mode selector — 12 routines (4 strategies × 3 J positions).
 *
 * <p>Select from a flat list on the Elastic dashboard. The Split Button Chooser
 * widget on the TeleDrive and Auto Setup tabs displays all options.
 */
public class AutoChooser {
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public AutoChooser(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    AutoRoutines r = new AutoRoutines(drivetrain, shooter, intake, indexer);

    // Default: highest scoring
    chooser.setDefaultOption("2-Cycle Depot Rush J-Center", r.twoCycleDepotRushCenter());

    // Depot Rush variants
    chooser.addOption("2-Cycle Depot Rush J-Left",  r.twoCycleDepotRushLeft());
    chooser.addOption("2-Cycle Depot Rush J-Right", r.twoCycleDepotRushRight());
    // Center Sweep variants
    chooser.addOption("Center Sweep J-Left",   r.centerSweepLeft());
    chooser.addOption("Center Sweep J-Center", r.centerSweepCenter());
    chooser.addOption("Center Sweep J-Right",  r.centerSweepRight());
    // Shoot + Mobility variants
    chooser.addOption("Shoot+Mobility J-Left",   r.shootAndLeaveLeft());
    chooser.addOption("Shoot+Mobility J-Center", r.shootAndLeaveCenter());
    chooser.addOption("Shoot+Mobility J-Right",  r.shootAndLeaveRight());
    // Shoot In Place variants
    chooser.addOption("Shoot In Place J-Left",   r.shootInPlaceLeft());
    chooser.addOption("Shoot In Place J-Center", r.shootInPlaceCenter());
    chooser.addOption("Shoot In Place J-Right",  r.shootInPlaceRight());

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}
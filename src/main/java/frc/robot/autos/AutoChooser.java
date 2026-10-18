package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Dashboard auto mode selector with 12 routines (4 types x 3 J positions).
 *
 * <p>Use the Split Button Chooser on Elastic to navigate:
 * <ul>
 *   <li>Pick your starting position: J-Left, J-Center, or J-Right
 *   <li>Pick your strategy: Depot Rush, Center Sweep, Shoot+Leave, or Shoot In Place
 * </ul>
 */
public class AutoChooser {
  private final AutoRoutines routines;
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public AutoChooser(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.routines = new AutoRoutines(drivetrain, shooter, intake, indexer);

    // ---- 2-Cycle Depot Rush ----
    chooser.addOption("Depot Rush J-Left", routines.twoCycleDepotRushLeft());
    chooser.addOption("Depot Rush J-Center", routines.twoCycleDepotRushCenter());
    chooser.addOption("Depot Rush J-Right", routines.twoCycleDepotRushRight());

    // ---- Center Sweep ----
    chooser.addOption("Center Sweep J-Left", routines.centerSweepLeft());
    chooser.addOption("Center Sweep J-Center", routines.centerSweepCenter());
    chooser.addOption("Center Sweep J-Right", routines.centerSweepRight());

    // ---- Shoot + Mobility ----
    chooser.addOption("Shoot+Leave J-Left", routines.shootAndLeaveLeft());
    chooser.addOption("Shoot+Leave J-Center", routines.shootAndLeaveCenter());
    chooser.addOption("Shoot+Leave J-Right", routines.shootAndLeaveRight());

    // ---- Shoot In Place ----
    chooser.addOption("Shoot In Place J-Left", routines.shootInPlaceLeft());
    chooser.setDefaultOption("Shoot In Place J-Center", routines.shootInPlaceCenter());
    chooser.addOption("Shoot In Place J-Right", routines.shootInPlaceRight());

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}
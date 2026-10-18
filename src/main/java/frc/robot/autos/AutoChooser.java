package frc.robot.autos;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Auto chooser that publishes a simple string to NetworkTables.
 *
 * <p>Elastic reads this from the topic directly. Use a "String Chooser"
 * widget in Elastic bound to {@code /SmartDashboard/Auto Chooser}.
 *
 * <p>When selected, publishes one of:
 * {@code Depot-Left}, {@code Depot-Center}, {@code Depot-Right},
 * {@code Sweep-Left}, {@code Sweep-Center}, {@code Sweep-Right},
 * {@code Leave-Left}, {@code Leave-Center}, {@code Leave-Right},
 * {@code InPlace-Left}, {@code InPlace-Center}, {@code InPlace-Right}.
 */
public class AutoChooser {
  private final AutoRoutines routines;
  private final SendableChooser<String> chooser = new SendableChooser<>();

  private String lastSelection = "";
  private Command lastCommand = Commands.none();

  public AutoChooser(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.routines = new AutoRoutines(drivetrain, shooter, intake, indexer);

    // Use a String chooser — Elastic handles these reliably
    chooser.setDefaultOption("2-Cycle Depot Rush J-Center", "Depot-Center");

    chooser.addOption("2-Cycle Depot Rush J-Left",  "Depot-Left");
    chooser.addOption("2-Cycle Depot Rush J-Right", "Depot-Right");

    chooser.addOption("Center Sweep J-Left",   "Sweep-Left");
    chooser.addOption("Center Sweep J-Center", "Sweep-Center");
    chooser.addOption("Center Sweep J-Right",  "Sweep-Right");

    chooser.addOption("Shoot+Mobility J-Left",   "Leave-Left");
    chooser.addOption("Shoot+Mobility J-Center", "Leave-Center");
    chooser.addOption("Shoot+Mobility J-Right",  "Leave-Right");

    chooser.addOption("Shoot In Place J-Left",   "InPlace-Left");
    chooser.addOption("Shoot In Place J-Center", "InPlace-Center");
    chooser.addOption("Shoot In Place J-Right",  "InPlace-Right");

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  /** Returns the command matching the currently selected string. */
  public Command getSelected() {
    String selection = chooser.getSelected();
    if (selection == null || selection.equals(lastSelection)) {
      return lastCommand;
    }
    lastSelection = selection;
    lastCommand = buildCommand(selection);
    return lastCommand;
  }

  private Command buildCommand(String key) {
    if (key == null) return routines.twoCycleDepotRushCenter();
    return switch (key) {
      case "Depot-Left"    -> routines.twoCycleDepotRushLeft();
      case "Depot-Center"  -> routines.twoCycleDepotRushCenter();
      case "Depot-Right"   -> routines.twoCycleDepotRushRight();
      case "Sweep-Left"    -> routines.centerSweepLeft();
      case "Sweep-Center"  -> routines.centerSweepCenter();
      case "Sweep-Right"   -> routines.centerSweepRight();
      case "Leave-Left"    -> routines.shootAndLeaveLeft();
      case "Leave-Center"  -> routines.shootAndLeaveCenter();
      case "Leave-Right"   -> routines.shootAndLeaveRight();
      case "InPlace-Left"  -> routines.shootInPlaceLeft();
      case "InPlace-Center" -> routines.shootInPlaceCenter();
      case "InPlace-Right" -> routines.shootInPlaceRight();
      default              -> routines.twoCycleDepotRushCenter();
    };
  }
}
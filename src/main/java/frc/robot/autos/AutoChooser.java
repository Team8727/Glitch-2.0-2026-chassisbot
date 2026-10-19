package frc.robot.autos;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Auto selector using a simple string on SmartDashboard.
 *
 * <p>Type the auto key into {@code /SmartDashboard/Auto Key} on Elastic.
 * Available keys (case-sensitive):
 * <pre>
 *   Depot-Left    Depot-Center    Depot-Right
 *   Sweep-Left    Sweep-Center    Sweep-Right
 *   Leave-Left    Leave-Center    Leave-Right
 *   InPlace-Left  InPlace-Center  InPlace-Right
 * </pre>
 *
 * Displays the human-readable label on {@code /Auto/label}.
 */
public class AutoChooser {
  private final AutoRoutines routines;
  private String lastKey = "";
  private Command lastCmd = Commands.none();

  public AutoChooser(Drivetrain dt, Shooter s, Intake i, Indexer ix) {
    this.routines = new AutoRoutines(dt, s, i, ix);

    // Publish available options as a reference
    NetworkTableInstance.getDefault()
        .getStringTopic("Auto/options").publish()
        .set("Depot-L/C/R  Sweep-L/C/R  Leave-L/C/R  InPlace-L/C/R");

    // Set default
    SmartDashboard.putString("Auto Key", "Depot-Center");
    NetworkTableInstance.getDefault()
        .getStringTopic("Auto/label").publish()
        .set("2-Cycle Depot Rush J-Center");
  }

  public Command getSelected() {
    String key = SmartDashboard.getString("Auto Key", "Depot-Center");
    if (key == null || key.isEmpty()) key = "Depot-Center";

    if (!key.equals(lastKey)) {
      lastKey = key;
      lastCmd = build(key);
      // Update label
      NetworkTableInstance.getDefault()
          .getStringTopic("Auto/label").publish()
          .set(label(key));
    }
    return lastCmd;
  }

  private static String label(String key) {
    return switch (key) {
      case "Depot-Left"    -> "Depot Rush J-Left";
      case "Depot-Center"  -> "Depot Rush J-Center";
      case "Depot-Right"   -> "Depot Rush J-Right";
      case "Sweep-Left"    -> "Center Sweep J-Left";
      case "Sweep-Center"  -> "Center Sweep J-Center";
      case "Sweep-Right"   -> "Center Sweep J-Right";
      case "Leave-Left"    -> "Shoot+Leave J-Left";
      case "Leave-Center"  -> "Shoot+Leave J-Center";
      case "Leave-Right"   -> "Shoot+Leave J-Right";
      case "InPlace-Left"  -> "Shoot In Place J-Left";
      case "InPlace-Center"-> "Shoot In Place J-Center";
      case "InPlace-Right" -> "Shoot In Place J-Right";
      default              -> "Unknown: " + key;
    };
  }

  private Command build(String key) {
    return switch (key) {
      case "Depot-Left"     -> routines.twoCycleDepotRushLeft();
      case "Depot-Center"   -> routines.twoCycleDepotRushCenter();
      case "Depot-Right"    -> routines.twoCycleDepotRushRight();
      case "Sweep-Left"     -> routines.centerSweepLeft();
      case "Sweep-Center"   -> routines.centerSweepCenter();
      case "Sweep-Right"    -> routines.centerSweepRight();
      case "Leave-Left"     -> routines.shootAndLeaveLeft();
      case "Leave-Center"   -> routines.shootAndLeaveCenter();
      case "Leave-Right"    -> routines.shootAndLeaveRight();
      case "InPlace-Left"   -> routines.shootInPlaceLeft();
      case "InPlace-Center" -> routines.shootInPlaceCenter();
      case "InPlace-Right"  -> routines.shootInPlaceRight();
      default               -> routines.twoCycleDepotRushCenter();
    };
  }
}
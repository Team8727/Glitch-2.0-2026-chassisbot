package frc.robot.autos;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Auto selector using raw NetworkTables strings only.
 *
 * <p>No SendableChooser — Elastic has trouble with those on some setups.
 * Instead, a StringPublisher writes the selection key, and you set it
 * from Elastic via a String Chooser widget bound to {@code /Auto/selection}.
 *
 * <p>Available keys (type into the Elastic widget or use the dashboard):
 * <ul>
 *   <li>{@code Depot-Left}, {@code Depot-Center}, {@code Depot-Right}</li>
 *   <li>{@code Sweep-Left}, {@code Sweep-Center}, {@code Sweep-Right}</li>
 *   <li>{@code Leave-Left}, {@code Leave-Center}, {@code Leave-Right}</li>
 *   <li>{@code InPlace-Left}, {@code InPlace-Center}, {@code InPlace-Right}</li>
 * </ul>
 */
public class AutoChooser {
  private final AutoRoutines routines;

  // Raw NT4 — no SendableChooser
  private final StringPublisher selectionPub;
  private final StringSubscriber selectionSub;

  private String lastKey = "";
  private Command lastCommand = Commands.none();
  private int frameCount = 0;

  public AutoChooser(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.routines = new AutoRoutines(drivetrain, shooter, intake, indexer);

    var inst = NetworkTableInstance.getDefault();
    selectionPub = inst.getStringTopic("Auto/selection").publish();
    selectionSub = inst.getStringTopic("Auto/selection").subscribe("Depot-Center");
    selectionPub.set("Depot-Center");

    // Also publish the human-readable label for the dashboard
    inst.getStringTopic("Auto/label").publish().set("2-Cycle Depot Rush J-Center");

    // Publish available options as a single string for reference
    inst.getStringTopic("Auto/options").publish().set(
        "Depot-Left, Depot-Center, Depot-Right, " +
        "Sweep-Left, Sweep-Center, Sweep-Right, " +
        "Leave-Left, Leave-Center, Leave-Right, " +
        "InPlace-Left, InPlace-Center, InPlace-Right");

    SmartDashboard.putString("Auto Key", "Depot-Center");
    SmartDashboard.putString("Auto Options",
        "Depot-Left | Depot-Center | Depot-Right | " +
        "Sweep-Left | Sweep-Center | Sweep-Right | " +
        "Leave-Left | Leave-Center | Leave-Right | " +
        "InPlace-Left | InPlace-Center | InPlace-Right");
  }

  public Command getSelected() {
    // Read from raw NT (settable from Elastic)
    String key = selectionSub.get();
    if (key == null || key.isEmpty()) key = "Depot-Center";

    // Also check SmartDashboard (settable from Glass/Shuffleboard)
    String sdKey = SmartDashboard.getString("Auto Key", "");
    if (!sdKey.isEmpty()) key = sdKey;

    if (!key.equals(lastKey)) {
      lastKey = key;
      lastCommand = buildCommand(key);
      // Update display
      selectionPub.set(key);
      NetworkTableInstance.getDefault()
          .getStringTopic("Auto/label").publish()
          .set(labelForKey(key));
      SmartDashboard.putString("Auto Key", key);
    }

    return lastCommand;
  }

  private static String labelForKey(String key) {
    return switch (key) {
      case "Depot-Left"    -> "2-Cycle Depot Rush J-Left";
      case "Depot-Center"  -> "2-Cycle Depot Rush J-Center";
      case "Depot-Right"   -> "2-Cycle Depot Rush J-Right";
      case "Sweep-Left"    -> "Center Sweep J-Left";
      case "Sweep-Center"  -> "Center Sweep J-Center";
      case "Sweep-Right"   -> "Center Sweep J-Right";
      case "Leave-Left"    -> "Shoot+Mobility J-Left";
      case "Leave-Center"  -> "Shoot+Mobility J-Center";
      case "Leave-Right"   -> "Shoot+Mobility J-Right";
      case "InPlace-Left"  -> "Shoot In Place J-Left";
      case "InPlace-Center" -> "Shoot In Place J-Center";
      case "InPlace-Right" -> "Shoot In Place J-Right";
      default              -> "Unknown";
    };
  }

  private Command buildCommand(String key) {
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
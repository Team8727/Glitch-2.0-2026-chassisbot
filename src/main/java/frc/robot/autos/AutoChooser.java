package frc.robot.autos;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Two-part auto selector using a String chooser (reliable on Elastic)
 * plus a raw NT4 string topic as a fallback for dashboard display.
 *
 * <p>Publishes to:
 * <ul>
 *   <li>{@code /SmartDashboard/Auto Chooser} — SendableChooser (use Split Button Chooser widget)</li>
 *   <li>{@code /Auto/selected} — plain string (use String Chooser widget as fallback)</li>
 * </ul>
 */
public class AutoChooser {
  private final AutoRoutines routines;
  private final SendableChooser<String> chooser = new SendableChooser<>();
  private final StringPublisher selectedPub;

  private String lastSelection = "";
  private Command lastCommand = Commands.none();

  /** Descriptive labels for each option key. */
  private static final String[] KEYS = {
    "Depot-Left", "Depot-Center", "Depot-Right",
    "Sweep-Left", "Sweep-Center", "Sweep-Right",
    "Leave-Left", "Leave-Center", "Leave-Right",
    "InPlace-Left", "InPlace-Center", "InPlace-Right"
  };
  private static final String[] LABELS = {
    "2-Cycle Depot Rush J-Left",
    "2-Cycle Depot Rush J-Center",
    "2-Cycle Depot Rush J-Right",
    "Center Sweep J-Left",
    "Center Sweep J-Center",
    "Center Sweep J-Right",
    "Shoot+Mobility J-Left",
    "Shoot+Mobility J-Center",
    "Shoot+Mobility J-Right",
    "Shoot In Place J-Left",
    "Shoot In Place J-Center",
    "Shoot In Place J-Right"
  };

  public AutoChooser(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.routines = new AutoRoutines(drivetrain, shooter, intake, indexer);

    // Publish a raw string topic Elastic can definitely read
    selectedPub = NetworkTableInstance.getDefault()
        .getStringTopic("Auto/selected").publish();
    selectedPub.set(LABELS[1]); // default: Depot-Center

    // Populate chooser
    chooser.setDefaultOption(LABELS[1], KEYS[1]);
    for (int i = 0; i < KEYS.length; i++) {
      if (i == 1) continue; // already set as default
      chooser.addOption(LABELS[i], KEYS[i]);
    }

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  public Command getSelected() {
    String selection = chooser.getSelected();
    if (selection == null) {
      return routines.twoCycleDepotRushCenter();
    }
    if (!selection.equals(lastSelection)) {
      lastSelection = selection;
      lastCommand = buildCommand(selection);
    }
    // Always publish the human-readable label
    selectedPub.set(labelForKey(lastSelection));
    return lastCommand;
  }

  private static String labelForKey(String key) {
    for (int i = 0; i < KEYS.length; i++) {
      if (KEYS[i].equals(key)) return LABELS[i];
    }
    return "Unknown";
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
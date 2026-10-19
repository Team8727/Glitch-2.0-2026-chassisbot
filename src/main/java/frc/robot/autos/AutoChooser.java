package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Auto selector with a flat SendableChooser.
 *
 * <p>Elastic: use a <b>Split Button Chooser</b> bound to
 * {@code /SmartDashboard/Auto Mode} to see all options as buttons.
 * Falls back to typing the key directly into {@code /SmartDashboard/Auto Key}
 * if the chooser doesn't render properly.
 */
public class AutoChooser {
  private final AutoRoutines routines;
  private final SendableChooser<String> chooser = new SendableChooser<>();

  private String lastKey = "";
  private Command lastCmd = null;

  public AutoChooser(Drivetrain dt, Shooter s, Intake i, Indexer ix) {
    this.routines = new AutoRoutines(dt, s, i, ix);

    // Populate chooser — these appear as buttons on Split Button Chooser
    chooser.setDefaultOption("Depot Rush J-Center", "Depot-Center");
    chooser.addOption("Depot Rush J-Left",   "Depot-Left");
    chooser.addOption("Depot Rush J-Right",  "Depot-Right");
    chooser.addOption("Center Sweep J-Left",  "Sweep-Left");
    chooser.addOption("Center Sweep J-Center","Sweep-Center");
    chooser.addOption("Center Sweep J-Right", "Sweep-Right");
    chooser.addOption("Shoot+Leave J-Left",   "Leave-Left");
    chooser.addOption("Shoot+Leave J-Center", "Leave-Center");
    chooser.addOption("Shoot+Leave J-Right",  "Leave-Right");
    chooser.addOption("In Place J-Left",      "InPlace-Left");
    chooser.addOption("In Place J-Center",    "InPlace-Center");
    chooser.addOption("In Place J-Right",     "InPlace-Right");

    // Publish to SmartDashboard (works with Elastic Split Button Chooser)
    SmartDashboard.putData("Auto Mode", chooser);

    // Also publish raw key as fallback (for Glass or typing)
    SmartDashboard.putString("Auto Key", "Depot-Center");

    // Publish human-readable label
    SmartDashboard.putString("Auto Label", "Depot Rush J-Center");
  }

  public Command getSelected() {
    // Try chooser first, fall back to raw string key
    String key = chooser.getSelected();
    if (key == null || key.isEmpty()) {
      key = SmartDashboard.getString("Auto Key", "Depot-Center");
    }

    if (!key.equals(lastKey)) {
      lastKey = key;
      lastCmd = build(key);
      // Keep fallback in sync
      SmartDashboard.putString("Auto Key", key);
      SmartDashboard.putString("Auto Label", label(key));
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
      case "InPlace-Left"  -> "In Place J-Left";
      case "InPlace-Center"-> "In Place J-Center";
      case "InPlace-Right" -> "In Place J-Right";
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
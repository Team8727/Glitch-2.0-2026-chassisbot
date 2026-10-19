package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Auto selector using a Number Slider (0-11) for reliable Elastic operation.
 *
 * <p>On the dashboard, add a <b>Number Slider</b> widget bound to
 * {@code /SmartDashboard/Auto Selector} with min=0, max=11, divisions=11.
 * A <b>Text Display</b> bound to {@code /SmartDashboard/Auto Label} shows
 * the human-readable name.
 *
 * <pre>
 *   0 = Depot-Left      1 = Depot-Center      2 = Depot-Right
 *   3 = Sweep-Left      4 = Sweep-Center      5 = Sweep-Right
 *   6 = Leave-Left      7 = Leave-Center      8 = Leave-Right
 *   9 = InPlace-Left   10 = InPlace-Center   11 = InPlace-Right
 * </pre>
 */
public class AutoChooser {
  private final AutoRoutines routines;
  private int lastIndex = -1;
  private Command lastCmd = Commands.none();

  private static final String[] LABELS = {
    "2-Cycle Depot Rush J-Left",   // 0
    "2-Cycle Depot Rush J-Center", // 1  (default)
    "2-Cycle Depot Rush J-Right",  // 2
    "Center Sweep J-Left",        // 3
    "Center Sweep J-Center",      // 4
    "Center Sweep J-Right",       // 5
    "Shoot+Leave J-Left",         // 6
    "Shoot+Leave J-Center",       // 7
    "Shoot+Leave J-Right",        // 8
    "In Place J-Left",            // 9
    "In Place J-Center",          // 10
    "In Place J-Right"            // 11
  };

  public AutoChooser(Drivetrain dt, Shooter s, Intake i, Indexer ix) {
    this.routines = new AutoRoutines(dt, s, i, ix);

    // Default: Depot-Center (index 1)
    SmartDashboard.putNumber("Auto Selector", 1);
    SmartDashboard.putString("Auto Label", LABELS[1]);

    // Publish reference map for drivers
    StringBuilder sb = new StringBuilder();
    for (int idx = 0; idx < LABELS.length; idx++) {
      sb.append(idx).append("=").append(LABELS[idx]).append("  ");
    }
    SmartDashboard.putString("Auto Options", sb.toString());
  }

  public Command getSelected() {
    int index = (int) SmartDashboard.getNumber("Auto Selector", 1);
    if (index < 0) index = 0;
    if (index > 11) index = 11;

    if (index != lastIndex) {
      lastIndex = index;
      lastCmd = build(index);
      SmartDashboard.putString("Auto Label", LABELS[index]);
    }
    return lastCmd;
  }

  private Command build(int index) {
    return switch (index) {
      case 0  -> routines.twoCycleDepotRushLeft();
      case 1  -> routines.twoCycleDepotRushCenter();
      case 2  -> routines.twoCycleDepotRushRight();
      case 3  -> routines.centerSweepLeft();
      case 4  -> routines.centerSweepCenter();
      case 5  -> routines.centerSweepRight();
      case 6  -> routines.shootAndLeaveLeft();
      case 7  -> routines.shootAndLeaveCenter();
      case 8  -> routines.shootAndLeaveRight();
      case 9  -> routines.shootInPlaceLeft();
      case 10 -> routines.shootInPlaceCenter();
      case 11 -> routines.shootInPlaceRight();
      default -> routines.twoCycleDepotRushCenter();
    };
  }
}
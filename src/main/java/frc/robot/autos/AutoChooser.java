package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

/**
 * Auto selector with three independent selection methods:
 *
 * <ol>
 *   <li><b>SendableChooser</b> published as "Auto Chooser" for Elastic/Glass.
 *       Use Elastic's <i>Split Button Chooser</i> widget.
 *   <li><b>Controller cycling (primary, dash-independent)</b> — while DISABLED,
 *       press the Left/Right bumpers to cycle autos, Start to announce the
 *       current selection via text on the dashboard.
 *   <li><b>Raw key</b> in {@code /SmartDashboard/Auto Key} for direct typing.
 * </ol>
 */
public class AutoChooser {
  private final AutoRoutines routines;
  private final SendableChooser<Integer> chooser = new SendableChooser<>();

  private int selection = 1; // default Depot-Center
  private Command lastCmd = Commands.none();

  private static final int COUNT = 12;
  private static final String[] LABELS = {
    "2-Cycle Depot Rush J-Left",   // 0
    "2-Cycle Depot Rush J-Center", // 1 (default)
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

  @SuppressWarnings("unused")
  private static final String[] KEYS = {
    "Depot-Left", "Depot-Center", "Depot-Right",
    "Sweep-Left", "Sweep-Center", "Sweep-Right",
    "Leave-Left", "Leave-Center", "Leave-Right",
    "InPlace-Left", "InPlace-Center", "InPlace-Right"
  };

  public AutoChooser(Drivetrain dt, Shooter s, Intake i, Indexer ix, CommandXboxController controller) {
    this.routines = new AutoRoutines(dt, s, i, ix);

    // Method 1: SendableChooser — the official approach.
    // Elastic's Split Button Chooser reads this. Keep the canonical name.
    chooser.setDefaultOption(LABELS[1], 1);
    for (int n = 0; n < COUNT; n++) {
      if (n == 1) continue;
      chooser.addOption(LABELS[n], n);
    }
    SmartDashboard.putData("Auto Chooser", chooser);

    // Method 2: Controller cycling while disabled (primary, dash-independent).
    if (controller != null) {
      RobotModeTriggers.disabled().onTrue(
        runOnce(() -> publish()).ignoringDisable(true));
      controller.rightBumper().and(RobotModeTriggers.disabled())
        .onTrue(runOnce(() -> cycle(1)).ignoringDisable(true));
      controller.leftBumper().and(RobotModeTriggers.disabled())
        .onTrue(runOnce(() -> cycle(-1)).ignoringDisable(true));
    }

    // Method 3: raw key fallback
    SmartDashboard.putString("Auto Key", KEYS[selection]);

    publish();
  }

  /** Advances the selection by delta (-1 or +1), wrapping around. */
  private void cycle(int delta) {
    selection = (selection + delta + COUNT) % COUNT;
    publish();
  }

  private void publish() {
    SmartDashboard.putString("Auto Label", LABELS[selection]);
    SmartDashboard.putString("Auto Key", KEYS[selection]);
    SmartDashboard.putNumber("Auto Index", selection);
  }

  /** Returns the currently selected autonomous command. */
  public Command getSelected() {
    // Priority: chooser selection (if changed on dashboard), then raw key, then current.
    Integer fromChooser = chooser.getSelected();
    if (fromChooser != null) {
      selection = Math.max(0, Math.min(COUNT - 1, fromChooser));
    } else {
      String key = SmartDashboard.getString("Auto Key", KEYS[selection]);
      for (int n = 0; n < COUNT; n++) {
        if (KEYS[n].equalsIgnoreCase(key)) { selection = n; break; }
      }
    }

    if (lastCmd == null || lastIndex != selection) {
      lastIndex = selection;
      lastCmd = build(selection);
      publish();
    }
    return lastCmd;
  }

  private int lastIndex = -1;

  /** Builds the command for the given auto index (0-11). */
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
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

import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * Auto selector — two independent methods:
 *
 * <ol>
 *   <li><b>SendableChooser</b> published as {@code "Auto Chooser"} for Elastic.
 *       Use the <i>Split Button Chooser</i> or drag the subtable onto your grid.
 *   <li><b>Controller cycling</b> while DISABLED: Left Bumper = previous,
 *       Right Bumper = next auto. Shows on dashboard as text.
 * </ol>
 */
public class AutoChooser {
  private final AutoRoutines routines;
  private final SendableChooser<Integer> chooser = new SendableChooser<>();

  private int selection = 1; // default: Depot-Center
  private Command lastCmd = Commands.none();
  private int lastIndex = -1;

  private static final String[] LABELS = {
    "Depot L", "Depot C", "Depot R",
    "Sweep L", "Sweep C", "Sweep R",
    "Leave L", "Leave C", "Leave R",
    "In L",    "In C",    "In R"
  };
  private static final String[] KEYS = {
    "Depot-Left", "Depot-Center", "Depot-Right",
    "Sweep-Left", "Sweep-Center", "Sweep-Right",
    "Leave-Left", "Leave-Center", "Leave-Right",
    "InPlace-Left", "InPlace-Center", "InPlace-Right"
  };

  public AutoChooser(Drivetrain dt, Shooter s, Intake i, Indexer ix, CommandXboxController controller) {
    this.routines = new AutoRoutines(dt, s, i, ix);

    // --- Method 1: SendableChooser for Elastic ---
    chooser.setDefaultOption("2-Cycle Depot Rush C", 1);
    for (int n = 0; n < 12; n++) if (n != 1) chooser.addOption(LABELS[n], n);
    SmartDashboard.putData("Auto Chooser", chooser);

    // --- Method 2: Controller cycling while disabled (no dashboard needed) ---
    if (controller != null) {
      controller.rightBumper().and(RobotModeTriggers.disabled())
        .onTrue(runOnce(() -> cycle(1)).ignoringDisable(true));
      controller.leftBumper().and(RobotModeTriggers.disabled())
        .onTrue(runOnce(() -> cycle(-1)).ignoringDisable(true));
    }

    publish();
  }

  private void cycle(int delta) {
    selection = (selection + delta + 12) % 12;
    publish();
  }

  private void publish() {
    SmartDashboard.putString("Auto Label", LABELS[selection]);
    SmartDashboard.putNumber("Auto Index", selection);
  }

  public Command getSelected() {
    // Read from chooser (primary)
    Integer fromChooser = chooser.getSelected();
    if (fromChooser != null) selection = Math.max(0, Math.min(11, fromChooser));

    if (lastCmd == null || selection != lastIndex) {
      lastIndex = selection;
      lastCmd = build(selection);
      publish();
    }
    return lastCmd;
  }

  private Command build(int idx) {
    return switch (idx) {
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
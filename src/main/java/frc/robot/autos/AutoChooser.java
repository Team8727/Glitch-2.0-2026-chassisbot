package frc.robot.autos;

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
 * Auto selector — controller cycling while disabled.
 *
 * <p>While the robot is DISABLED:
 * <ul>
 *   <li><b>Right Bumper</b> — next auto
 *   <li><b>Left Bumper</b> — previous auto
 * </ul>
 *
 * <p>The current selection shows on the dashboard as a text label.
 * No Elastic widgets needed. Also publishes a raw string key for Glass/Shuffleboard.
 */
public class AutoChooser {
  private final AutoRoutines routines;
  private int selection = 1; // default Depot-Center
  private Command lastCmd = Commands.none();

  // Labels for human readability
  private static final String[] LABELS = {
    "Depot L", "Depot C", "Depot R",
    "Sweep L", "Sweep C", "Sweep R",
    "Leave L", "Leave C", "Leave R",
    "In L",    "In C",    "In R"
  };
  // Raw keys for Glass/Shuffleboard
  private static final String[] KEYS = {
    "Depot-Left", "Depot-Center", "Depot-Right",
    "Sweep-Left", "Sweep-Center", "Sweep-Right",
    "Leave-Left", "Leave-Center", "Leave-Right",
    "InPlace-Left", "InPlace-Center", "InPlace-Right"
  };

  public AutoChooser(Drivetrain dt, Shooter s, Intake i, Indexer ix, CommandXboxController controller) {
    this.routines = new AutoRoutines(dt, s, i, ix);

    // Controller cycling while disabled — works without any dashboard
    if (controller != null) {
      controller.rightBumper().and(RobotModeTriggers.disabled())
        .onTrue(runOnce(() -> cycle(1)).ignoringDisable(true));
      controller.leftBumper().and(RobotModeTriggers.disabled())
        .onTrue(runOnce(() -> cycle(-1)).ignoringDisable(true));
    }

    // Publish to dashboard
    SmartDashboard.putString("Auto Label", LABELS[selection]);
    SmartDashboard.putString("Auto Key", KEYS[selection]);
    SmartDashboard.putString("Auto Options",
      "0=DepotL 1=DepotC 2=DepotR 3=SweepL 4=SweepC 5=SweepR "
      + "6=LeaveL 7=LeaveC 8=LeaveR 9=InL 10=InC 11=InR");
  }

  private void cycle(int delta) {
    selection = (selection + delta + 12) % 12;
    SmartDashboard.putString("Auto Label", LABELS[selection]);
    SmartDashboard.putString("Auto Key", KEYS[selection]);
  }

  /** Returns the currently selected autonomous command. */
  public Command getSelected() {
    return build(selection);
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
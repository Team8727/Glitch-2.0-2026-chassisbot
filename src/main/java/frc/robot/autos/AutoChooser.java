package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Dashboard auto mode selector.
 *
 * <p>Exposes a {@link SendableChooser} on SmartDashboard that lets the driver
 * select which autonomous routine to run.
 */
public class AutoChooser {
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  private final AutoRoutines routines;

  public AutoChooser(
      Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
    this.routines = new AutoRoutines(drivetrain, shooter, intake, indexer);

    chooser.setDefaultOption("4-Piece Auto", routines.fourPieceAuto());
    chooser.addOption("2-Piece + Leave", routines.twoPieceAuto());
    chooser.addOption("Shoot + Leave", routines.shootAndLeave());
    chooser.addOption("Shoot In Place", routines.shootInPlace());
    chooser.addOption("Simple 2-Piece", routines.simpleTwoPiece());

    SmartDashboard.putData("Auto Chooser", chooser);
  }

  /** @return The currently selected autonomous command. */
  public Command getSelected() {
    return chooser.getSelected();
  }
}
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.ShooterPivot;
import frc.robot.Subsystems.ShooterRoller;

public class IndexAndShoot extends Command {

  Indexer indexer;
  ShooterPivot shooterPivot;
  ShooterRoller shooterRoller;

  public IndexAndShoot(Indexer indexer, ShooterPivot shooterPivot, ShooterRoller shooterRoller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, shooterPivot, shooterRoller);
    this.indexer = indexer;
    this.shooterPivot = shooterPivot;
    this.shooterRoller = shooterRoller;
  }

  @Override
  public void initialize() {
    // This method will be called once per scheduler run

    // point robot
    // point shooter pivot
    // spin up indexer
    // spin up shooter
  }

  @Override
  public void execute() {
    // This method will be called repeatedly until this Command either finishes or is interrupted.

    // adjust robot yaw
    // adjust shooter pitch
  }

  @Override
  public void end(boolean interrupted) {
    // This method will be called once after isFinished returns true, or if the command is interrupted/canceled.

    // stop shooter
    // stop indexer
  }

  @Override
  public boolean isFinished() {
    // Return true when this Command no longer needs to run execute(). Returning false will result in this Command being scheduled forever.
    return false;
  }

}
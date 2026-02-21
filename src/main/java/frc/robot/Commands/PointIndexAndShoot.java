package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drivetrain.CTRESwerveDrivetrain;
import frc.robot.Robot;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.ShooterPivot;
import frc.robot.Subsystems.ShooterRoller;
import frc.robot.controller.CTReSwerveControls;

import static frc.robot.controller.CTReSwerveControls.MaxSpeed;

public class PointIndexAndShoot extends Command {

  CommandXboxController controller;

  Indexer indexer;
  ShooterPivot shooterPivot;
  ShooterRoller shooterRoller;
  CTRESwerveDrivetrain drivetrain;

  public PointIndexAndShoot(Indexer indexer, ShooterPivot shooterPivot, ShooterRoller shooterRoller, CTRESwerveDrivetrain drivetrain, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, shooterPivot, shooterRoller, drivetrain);
    this.controller = controller;
    this.indexer = indexer;
    this.shooterPivot = shooterPivot;
    this.shooterRoller = shooterRoller;
    this.drivetrain = drivetrain;

  }

  @Override
  public void initialize() {  // This method will be called once per scheduler run
    // 1. point robot
    pointRobot();
    // 2. point shooter pivot
    angleShooterPivot();
    // 3. spin up indexer
    indexer.setSpeedDutyCycle(1); // TODO: find right value for speed of indexer to feed balls into shooter without jamming or leaving too much space between balls
    // 4. spin up shooter
    shooterRoller.setSpeedDutyCycle(Robot.firing.power);
  }

  @Override
  public void execute() {  // This method will be called repeatedly until this Command either finishes or is interrupted.
    // 5. adjust robot yaw
    pointRobot();
    // 6. adjust shooter pitch
    angleShooterPivot();
  }

  @Override
  public void end(boolean interrupted) {  // This method will be called once after isFinished returns true, or if the command is interrupted/canceled.
    // 7. stop shooter
    shooterRoller.setSpeedDutyCycle(0);
    // 8. stop indexer
    indexer.setSpeedDutyCycle(0);
  }

  @Override
  public boolean isFinished() {  // Return true when this Command no longer needs to run execute(). Returning false will result in this Command being scheduled forever.
    return false;
  }

  private void pointRobot() {
    drivetrain.applyRequest(() -> {
      return CTReSwerveControls.faceTarget
              .withTargetDirection(Rotation2d.fromDegrees(Robot.firing.yaw)) // face the target with 180-degree offset I had to add for some reason
              .withVelocityX(-controller.getLeftY() * MaxSpeed) // translate across field (driving from red to blue alliance sides)
              .withVelocityY(-controller.getLeftX() * MaxSpeed); // translate across field (driving from field long wall to other long wall)
    });
  }

  private void angleShooterPivot() {
    if (Robot.firing.pitch < ShooterPivot.MaxShooterAngles.UP.getDegrees() && Robot.firing.pitch > ShooterPivot.MaxShooterAngles.DOWN.getDegrees()) {
      shooterPivot.setPosition(Robot.firing.pitch);
    } else {
      controller.setRumble(GenericHID.RumbleType.kRightRumble, 1); // rumble controller if trying to set shooter pivot below minimum angle or above maximum angle to alert driver that they are trying to set an invalid angle
    }
  }

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakePivot;
import frc.robot.Subsystems.IntakeRoller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinIntakeCmd extends Command {

  private final IntakeRoller roller;
  private final IntakePivot pivot;

  boolean ended = false;

  /** Creates a new IntakeRollerCommand. */
  public SpinIntakeCmd(IntakeRoller roller, IntakePivot pivot) {

    this.roller = roller;
    this.pivot = pivot;

    addRequirements(roller, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (pivot.getPosition() == IntakePivot.IntakePosition.DOWN.getDegrees()) {
      roller.setSpeedDutyCycle(1);
    } else {
      roller.setSpeedDutyCycle(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roller.setSpeedDutyCycle(0);
    ended = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}

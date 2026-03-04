// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakePivot;
import frc.robot.Subsystems.IntakeRoller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseIntakeCmd extends Command {

  private final IntakeRoller roller;
  private final IntakePivot pivot;

  boolean ended = false;

  /** Creates a new IntakeRollerCommand. */
  public RaiseIntakeCmd(IntakeRoller roller, IntakePivot pivot) {
    this.roller = roller;
    this.pivot = pivot;
    addRequirements(roller, pivot);
    System.out.println("RaiseIntakeCmd");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    roller.setSpeedDutyCycle(0);
    if (Math.abs(pivot.getPosition() * 360 - IntakePivot.IntakePosition.MID.getDegrees()) < 5) {
      pivot.setPosition(IntakePivot.IntakePosition.DOWN.getDegrees());
      System.out.println("intake down");
      cancel();
    } else {
      pivot.setPosition(IntakePivot.IntakePosition.MID.getDegrees());
      System.out.println("intake up");
      cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

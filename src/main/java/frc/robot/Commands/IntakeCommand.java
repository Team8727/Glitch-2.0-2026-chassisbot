package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.IntakeRoller;

public class IntakeCommand extends SequentialCommandGroup {
  public IntakeCommand(IntakeRoller intake) {
    addCommands(
            intake.run(() -> {
                      SmartDashboard.putBoolean("command ran", true);
                      intake.setDutyCycle(.8);
                    })
                    .finallyDo(() -> {
                      intake.setDutyCycle(0);
                    })
    );
  }
}

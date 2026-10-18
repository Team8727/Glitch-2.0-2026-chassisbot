package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;

import static edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * Intake command with optional unjam pulse.
 *
 * <p>Starts with a brief reverse pulse to clear jams, then runs intake + indexer
 * forward. The unjam duration is dashboard-tunable via "Intake Unjam Time".
 *
 * <p>When the trigger is released, both intake and indexer stop.
 */
public class IntakeCommand extends SequentialCommandGroup {

    private static final double kUnjamTime = 0.15; // seconds
    private static final double kIntakeTimeout = 10.0; // safety timeout

    public IntakeCommand(Intake intake, Indexer indexer) {
        SmartDashboard.putNumber("Intake Unjam Time", kUnjamTime * 1000); // ms

        addCommands(
                // Brief reverse pulse to clear any jams
                runOnce(() -> {
                    double unjamMs = SmartDashboard.getNumber("Intake Unjam Time", 150.0);
                    intake.setDutyCycle(-Constants.kIntakeDutyCycle * 0.6);
                    indexer.setDutyCycle(-Constants.kIndexerForwardDutyCycle * 0.3);
                }),
                waitSeconds(kUnjamTime),
                // Stop reverse
                runOnce(() -> { intake.setDutyCycle(0); indexer.setDutyCycle(0); }),
                waitSeconds(0.05),
                // Start forward intake (runs until canceled)
                runOnce(() -> {
                    intake.setDutyCycle(Constants.kIntakeDutyCycle);
                    indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
                    intake.setIntaking(true);
                    LED.instance.intakePatterns();
                })
        );
    }
}
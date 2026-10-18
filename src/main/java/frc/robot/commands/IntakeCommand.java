package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.LED;

/**
 * Simple intake command that runs the intake and indexer forward.
 *
 * <p>Sets the intake duty cycle and indexer duty cycle, enables the intaking
 * flag, and triggers LED patterns. On end, stops all mechanisms.
 */
public class IntakeCommand extends Command {
    private final Intake intake;
    private final Indexer indexer;

    /**
     * Creates a new IntakeCommand.
     *
     * @param intake  the intake subsystem
     * @param indexer the indexer subsystem
     */
    public IntakeCommand(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake, indexer);
    }

    @Override
    public void initialize() {
        intake.setDutyCycle(Constants.kIntakeDutyCycle);
        indexer.setDutyCycle(Constants.kIndexerForwardDutyCycle);
        intake.setIntaking(true);
        LED.instance.intakePatterns();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setDutyCycle(0);
        indexer.setDutyCycle(0);
        intake.setIntaking(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
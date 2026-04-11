package frc.robot.Subsystems;

import Glitch.Lib.LEDs.AbstractLEDS;
import Glitch.Lib.LEDs.GlitchLEDPatterns;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

public class LEDSubsystem extends AbstractLEDS {

    public final Section leftSide;
    public final Section pip;
    public final Section rightSide;

    public LEDSubsystem() {
        super(91, 40, 11, -40);

        leftSide = getSections().get(0);
        pip = getSections().get(1);
        pip.setBase(LEDPattern.kOff);
        rightSide = getSections().get(2);
    }

    static LEDSubsystem instance;

    public static LEDSubsystem getInstance() {
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }

    public void start() {
         leftSide.setBase(GlitchLEDPatterns.fire(LEDPattern.solid(Color.kGreen), Color.kBlack));
        // leftSide.setBase(GlitchLEDPatterns.rainDrops(LEDPattern.solid(Color.kBlue), 1, 1));
        leftSide.setPattern(GlitchLEDPatterns.purple, 2);
        pip.setBase(GlitchLEDPatterns.purple);
        pip.setPattern(GlitchLEDPatterns.purple);
        rightSide.setBase(GlitchLEDPatterns.fire(LEDPattern.solid(Color.kGreen), Color.kBlack));
        // rightSide.setBase(GlitchLEDPatterns.rainDrops(LEDPattern.solid(Color.kBlue), 1, 1));
        rightSide.setPattern(GlitchLEDPatterns.purple, 2);
    }

    public void autoInit() {
        leftSide.setBase(GlitchLEDPatterns.fire(LEDPattern.solid(Color.kRed), Color.kBlack));
        pip.setBase(LEDPattern.solid(Color.kOrangeRed));
        rightSide.setBase(GlitchLEDPatterns.fire(LEDPattern.solid(Color.kRed), Color.kBlack));
        leftSide.setPattern(LEDPattern.solid(Color.kOrangeRed), 0.5);
        pip.setPattern(LEDPattern.solid(Color.kOrangeRed), 0.5);
        rightSide.setPattern(LEDPattern.solid(Color.kOrangeRed), 0.5);
    }

    public void teleopInit() {
        leftSide.setBase(GlitchLEDPatterns.fire(LEDPattern.solid(Color.kRed), Color.kBlack));
        pip.setBase(LEDPattern.solid(Color.kGreen));
        rightSide.setBase(GlitchLEDPatterns.fire(LEDPattern.solid(Color.kRed), Color.kBlack));
        leftSide.setPattern(LEDPattern.solid(Color.kGreen), 0.5);
        pip.setPattern(LEDPattern.solid(Color.kGreen), 0.5);
        rightSide.setPattern(LEDPattern.solid(Color.kGreen), 0.5);
    }

    public void shootPatterns(double currentFlywheelVelocity, double targetFlywheelVelocity) {
        leftSide.setPattern(GlitchLEDPatterns.linearProgress(GlitchLEDPatterns.sunsetAce.scrollAtRelativeSpeed(Percent.per(Second).of(100)), currentFlywheelVelocity, targetFlywheelVelocity));
        rightSide.setPattern(GlitchLEDPatterns.linearProgress(GlitchLEDPatterns.sunsetAce.scrollAtRelativeSpeed(Percent.per(Second).of(100)), currentFlywheelVelocity, targetFlywheelVelocity));
    }

    public void endCommand() {
        leftSide.setPattern(GlitchLEDPatterns.blinkyGreen, 0);
        rightSide.setPattern(GlitchLEDPatterns.blinkyGreen, 0);
    }
    
    public void intakePatterns() {
        leftSide.setPattern(LEDPattern.gradient(GradientType.kContinuous, Color.kGreen, Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(100)));
        rightSide.setPattern(LEDPattern.gradient(GradientType.kContinuous, Color.kGreen, Color.kWhite).scrollAtRelativeSpeed(Percent.per(Second).of(100)));
    }

    // Stuff I stole from ShootComand so the LEDs would be accurate :|
    double speedCoefficient = 1 / (Math.PI * Robot.SHOOTER_FLYWHEEL_DIAMETER_METERS);
    double flywheelSpeed = speedCoefficient * (Robot.firing != null ? Robot.firing.power : 45);
    public double motorSpeed = flywheelSpeed * (24.0 /15);

    @Override
    public void periodic() {
        super.periodic();
    }
}
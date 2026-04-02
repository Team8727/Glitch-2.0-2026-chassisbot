package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import Glitch.Lib.LEDs.AbstractLEDS;
import Glitch.Lib.LEDs.GlitchLEDPatterns;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSubsystem extends AbstractLEDS {

    public final Section leftSide;
    public final Section pip;
    public final Section rightSide;

    public LEDSubsystem() {
        super(67, 30, 7, -30);

        leftSide = getSections().get(0);
        pip = getSections().get(1);
        pip.setBase(LEDPattern.kOff);
        rightSide = getSections().get(2);
    }
    
    public void start() {
        leftSide.setBase(GlitchLEDPatterns.randomNoise(GlitchLEDPatterns.sunsetAce).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
        leftSide.setPattern(GlitchLEDPatterns.purple, 2);
        pip.setBase(GlitchLEDPatterns.purple);
        pip.setPattern(GlitchLEDPatterns.purple);
        rightSide.setBase(GlitchLEDPatterns.randomNoise(GlitchLEDPatterns.sunsetAce).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
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

    @Override
    public void periodic() {
        super.periodic();
    }
}

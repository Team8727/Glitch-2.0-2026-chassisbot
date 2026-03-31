package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import Glitch.Lib.LEDs.AbstractLEDS;
import Glitch.Lib.LEDs.GlitchLEDPatterns;
import edu.wpi.first.wpilibj.LEDPattern;

public class LEDSubsystem extends AbstractLEDS {

    public final Section leftSide;
    public final Section pip;
    public final Section rightSide;

    public LEDSubsystem() {
        super(67, 30, 7, -30);

        leftSide = getSections().get(0);
        leftSide.setBase(GlitchLEDPatterns.randomNoise(GlitchLEDPatterns.sunsetAce).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
        pip = getSections().get(1);
        pip.setBase(LEDPattern.kOff);
        rightSide = getSections().get(2);
        rightSide.setBase(GlitchLEDPatterns.randomNoise(GlitchLEDPatterns.sunsetAce).scrollAtRelativeSpeed(Percent.per(Second).of(25)));
    }
    
    public void start() {
        leftSide.setPattern(GlitchLEDPatterns.purple, 2);
        pip.setPattern(GlitchLEDPatterns.purple);
        rightSide.setPattern(GlitchLEDPatterns.purple, 2);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}

//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.robot.Subsystems;
//
//import Glitch.Lib.LEDs.AbstractLEDS;
//import Glitch.Lib.LEDs.GlitchLEDPatterns;
//import edu.wpi.first.wpilibj.LEDPattern;
//import edu.wpi.first.wpilibj.util.Color;
//
//public class LEDSubsystem2026 extends AbstractLEDS {
//  /** Creates a new LEDSubsystem2026. */
//
//  public final Section bottomLeft;
//  public final Section shortLeft;
//  public final Section topLeft;
//  public final Section topRight;
//  public final Section shortRight;
//  public final Section bottomRight;
//
//  private LEDSubsystem2026() { //TODO: SET THE GODDAMN PWM VALUE BEFORE TESTING ON AN ACTUAL ROBOT
//    super(283, 65, 38, 40, -40, -34, -66);
//
//    bottomLeft = getSections().get(0);
//    bottomLeft.setBase(GlitchLEDPatterns.fire(GlitchLEDPatterns.theCoolerGreen, Color.kBlack));
//
//    shortLeft = getSections().get(1);
//    shortLeft.setBase(GlitchLEDPatterns.enzoMap.NORMAL.getEnzoMap());
//
//    topLeft = getSections().get(2);
//    topLeft.setBase(GlitchLEDPatterns.randomNoise(GlitchLEDPatterns.blinkyGreen));
//
//    topRight = getSections().get(3);
//    topRight.setBase(GlitchLEDPatterns.randomNoise(GlitchLEDPatterns.blinkyGreen));
//
//    shortRight = getSections().get(4);
//    shortRight.setBase(GlitchLEDPatterns.enzoMap.NORMAL.getEnzoMap());
//
//    bottomRight = getSections().get(5);
//    bottomRight.setBase(GlitchLEDPatterns.fire(GlitchLEDPatterns.theCoolerGreen, Color.kBlack));
//  }
//
//  static LEDSubsystem2026 instance;
//
//  public static LEDSubsystem2026 getInstance() {
//    if (instance != null) {
//      return instance;
//    } else {
//      instance = new LEDSubsystem2026();
//      return instance;
//    }
//  }
//
//  public void setAll(LEDPattern pattern, double durationSeconds) {
//    bottomLeft.setPattern(pattern, durationSeconds);
//    shortLeft.setPattern(pattern, durationSeconds);
//    topLeft.setPattern(pattern, durationSeconds);
//    topRight.setPattern(pattern, durationSeconds);
//    shortRight.setPattern(pattern, durationSeconds);
//    bottomRight.setPattern(pattern, durationSeconds);
//  }
//
//  public void setAll(LEDPattern pattern) {
//    setAll(pattern, Section.infiniteDurationSeconds);
//  }
//
//  public void returnAllToBase() {
//    bottomLeft.setPattern(bottomLeft.basePattern);
//    shortLeft.setPattern(shortLeft.basePattern);
//    topLeft.setPattern(topLeft.basePattern);
//    topRight.setPattern(topRight.basePattern);
//    shortRight.setPattern(shortRight.basePattern);
//    bottomRight.setPattern(bottomRight.basePattern);
//  }
//
//  @Override
//  public void periodic() {
//    // This method will be called once per scheduler run
//    super.periodic();
//  }
//}

package frc;

import Glitch.Lib.Motors.Motor;

public class TestModules {
  public static class FakeMotor implements Motor {
    public double positionRotations = 0.0; // rotations
    double velocity = 0.0; // rotations/sec
    double current = 0.0; // amps
    public double duty = 0.0; // -1..1
    double lastFeedforward = 0.0;

    @Override
    public void setVelocity(double speed) {
      this.velocity = speed;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
      this.duty = dutyCycle;
    }

    @Override
    public void setPosition(double position, double feedforward) {
      // For testing, snap immediately to requested position
      this.positionRotations = position;
      this.lastFeedforward = feedforward;
    }

    @Override
    public void setPosition(double position) {
      this.positionRotations = position;
    }

    @Override
    public double getPosition() {
      return positionRotations;
    }

    @Override
    public double getCurrent() {
      return current;
    }

    @Override
    public double getVelocity() {
      return velocity;
    }

    @Override
    public boolean getForwardLimitSwitch() {
      return false;
    }

    @Override
    public boolean getReverseLimitSwitch() {
      return false;
    }
  }

}

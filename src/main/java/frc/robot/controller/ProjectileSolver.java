package frc.robot.controller;

import edu.wpi.first.math.geometry.Translation3d;

public class ProjectileSolver {

  public static class FiringSolution {
    public double yaw;      // Rotation around Z (degrees) - Standard FRC Gyro frame
    public double pitch;    // Angle above ground (degrees)
    public double power;    // Muzzle velocity magnitude
    public boolean isValid;

    @Override
    public String toString() {
      return String.format("Valid: %b | Yaw: %.2f | Pitch: %.2f | Power: %.2f", isValid, yaw, pitch, power);
    }
  }

  public static FiringSolution solve(Translation3d start, Translation3d target, Translation3d shooterVel, double entryAngle) {
    FiringSolution sol = new FiringSolution();
    double g = 9.81;

    // 1. Calculate Differences (Standard WPILib: Z is Up)
    Translation3d diff = target.minus(start);

    // Horizontal distance (Hypotenuse of X and Y)
    // We ignore Z here to get ground distance
    double groundDist = Math.sqrt(diff.getX() * diff.getX() + diff.getY() * diff.getY());

    // Vertical height is Z
    double height = diff.getZ();

    // --- STEP 1: CALCULATE REQUIRED WORLD TRAJECTORY ---
    double phi = Math.toRadians(entryAngle);

    // 1a. Find required Launch Angle (theta)
    double tanTheta = (2 * height / groundDist) - Math.tan(phi);
    double theta = Math.atan(tanTheta);

    // 1b. Find required World Velocity Magnitude
    double cosTheta = Math.cos(theta);
    double denominator = 2 * Math.pow(cosTheta, 2) * (groundDist * tanTheta - height);

    if (denominator <= 0.0001) {
      sol.isValid = false;
      return sol;
    }

    double vWorldMag = Math.sqrt((g * Math.pow(groundDist, 2)) / denominator);

    // 1c. Construct the World Velocity Vector
    // Direction on ground is just X/Y normalized
    double groundDirX = diff.getX() / groundDist;
    double groundDirY = diff.getY() / groundDist;

    // Recombine: Horizontal parts scale by cos(theta), Vertical part (Z) is sin(theta)
    Translation3d vWorld = new Translation3d(
      groundDirX * (vWorldMag * cosTheta), // X component
      groundDirY * (vWorldMag * cosTheta), // Y component
      vWorldMag * Math.sin(theta)          // Z component (Vertical)
    );

    // --- STEP 2: COMPENSATE FOR SHOOTER MOVEMENT ---
    Translation3d vMuzzle = vWorld.minus(shooterVel);

    // --- STEP 3: EXTRACT YAW, PITCH, AND POWER ---
    sol.power = vMuzzle.getNorm();
    sol.isValid = true;

    // Yaw: Standard FRC (0 = +X, Left = +Y)
    sol.yaw = Math.toDegrees(Math.atan2(vMuzzle.getY(), vMuzzle.getX()));

    // Pitch: Calculate raw physics pitch (-90 to +90)
    // Positive = Up, Negative = Down
    double rawPitch = Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, vMuzzle.getZ() / sol.power))));

    // FIX: If you are seeing 110 when you expect 70, your system likely
    // measures pitch from the opposite horizon (180).
    // Try one of these fixes based on what your logger shows:

    // OPTION A: If you see 110 and want 70 (The "Supplementary" Flip)
    // This happens if 0 is "Backward" or the axis is reversed.
    sol.pitch = 180.0 - rawPitch;

    // OPTION B: If you just need to invert Up/Down (Simple Invert)
    // sol.pitch = -rawPitch;

    // OPTION C: Standard (Use this if you want 0=Flat, 90=Up)
    // sol.pitch = rawPitch;

    return sol;  }
}
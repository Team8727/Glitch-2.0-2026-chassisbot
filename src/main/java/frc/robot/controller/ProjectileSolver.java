package frc.robot.controller;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class ProjectileSolver {

  public static class FiringSolution {
    public double yaw;      // Rotation around Z (degrees) - Standard FRC Gyro frame
    public double pitch;    // Angle above ground (degrees)
    public double power;    // Muzzle velocity magnitude
    public boolean isValid; // Whether a valid solution was found (e.g. not out of range)
    public Translation3d worldVel;

    @Override
    public String toString() {
      return String.format("Valid: %b | Yaw: %.2f | Pitch: %.2f | Power: %.2f", isValid, yaw, pitch, power);
    }
  }

  public static FiringSolution solve(Translation3d start, Translation3d target, Translation3d shooterVel, double muzzlePitchDegrees) {
    FiringSolution sol = new FiringSolution();
    double g = 9.81;

    // The user requested the shooter pitch be locked at 60 degrees.
    // However, we use the parameter as requested, defaulting to 60 if needed.
    // In this project, pitch is measured above the horizontal.
    double pitchAngle = Math.toRadians(muzzlePitchDegrees);
    
    // Safety: if the user passes 0 or something weird, we stick to 60 as requested by the issue.
    if (Math.abs(muzzlePitchDegrees) < 0.1) pitchAngle = Math.toRadians(60.0);
    
    // We lock it to 60 as per the requirement "it should shoot at 60 degrees always"
    pitchAngle = Math.toRadians(60.0);
    double sinTheta = Math.sin(pitchAngle);
    double cosTheta = Math.cos(pitchAngle);
    double tanTheta = Math.tan(pitchAngle);
    double cotTheta = 1.0 / tanTheta;
    double cot2Theta = cotTheta * cotTheta;

    // Target relative to shooter at t=0
    Translation3d diff = target.minus(start);
    double dx = diff.getX();
    double dy = diff.getY();
    double dz = diff.getZ();

    // Robot velocity (field frame)
    double vrx = shooterVel.getX();
    double vry = shooterVel.getY();
    double vrz = shooterVel.getZ();

    // Projectile motion with moving platform:
    // We need to find time of flight 't' such that the muzzle velocity vector
    // (relative to the robot) has the required pitch (60 deg).
    //
    // Components of muzzle velocity Vm:
    // vmx = dx/t - vrx
    // vmy = dy/t - vry
    // vmz = (dz + 0.5*g*t^2)/t - vrz
    //
    // Constraint: vmz / sqrt(vmx^2 + vmy^2) = tan(60)
    // or: vmx^2 + vmy^2 = cot^2(60) * vmz^2
    //
    // Multiply by t^2:
    // (dx - vrx*t)^2 + (dy - vry*t)^2 = cot^2(60) * (dz + 0.5*g*t^2 - vrz*t)^2
    
    double a = 0.5 * g;
    double b = -vrz;
    double c = dz;
    double k = cot2Theta;

    // Coefficients for the quartic: A*t^4 + B*t^3 + C*t^2 + D*t + E = 0
    // RHS = k * (a*t^2 + b*t + c)^2 = k*(a^2 t^4 + 2ab t^3 + (b^2 + 2ac) t^2 + 2bc t + c^2)
    // LHS = (vrx^2 + vry^2) t^2 - 2(dx*vrx + dy*vry) t + (dx^2 + dy^2)
    
    double A = k * (a * a);
    double B = k * (2 * a * b);
    double C = k * (b * b + 2 * a * c) - (vrx * vrx + vry * vry);
    double D = k * (2 * b * c) + 2 * (dx * vrx + dy * vry);
    double E = k * (c * c) - (dx * dx + dy * dy);

    // Initial guess: Static shot time of flight
    double groundDist = Math.sqrt(dx * dx + dy * dy);
    double t;
    if (groundDist * tanTheta > dz) {
      t = Math.sqrt(2 * (groundDist * tanTheta - dz) / g);
    } else {
      t = 0.5; // Reasonable default for typical distances
    }

    // Newton-Raphson iteration
    for (int i = 0; i < 25; i++) {
      double t2 = t * t;
      double t3 = t2 * t;
      double t4 = t3 * t;

      double f = A * t4 + B * t3 + C * t2 + D * t + E;
      double fp = 4 * A * t3 + 3 * B * t2 + 2 * C * t + D;

      if (Math.abs(fp) < 1e-9) break;

      double delta = f / fp;
      t -= delta;

      if (Math.abs(delta) < 1e-7) break;
      if (t < 0.001) t = 0.001;
    }

    if (t <= 0.001 || Double.isNaN(t)) {
      sol.isValid = false;
      sol.worldVel = new Translation3d();
      return sol;
    }

    // Muzzle vertical speed relative to robot
    double vmz = (dz + 0.5 * g * t * t) / t - vrz;
    
    // Muzzle speed magnitude
    double s = vmz / sinTheta;

    if (s < 0) {
      sol.isValid = false;
      sol.worldVel = new Translation3d();
      return sol;
    }

    sol.power = s;
    sol.pitch = Math.toDegrees(pitchAngle);
    
    double vmx = dx / t - vrx;
    double vmy = dy / t - vry;
    sol.yaw = Math.toDegrees(Math.atan2(vmy, vmx));
    sol.isValid = true;
    
    // World velocity for reference
    sol.worldVel = new Translation3d(vmx + vrx, vmy + vry, vmz + vrz);

    return sol;
  }
}

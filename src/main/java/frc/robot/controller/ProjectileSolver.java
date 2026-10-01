package frc.robot.controller;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Solves for projectile motion parameters to hit a target from a moving shooter
 * at a fixed launch pitch angle.
 *
 * <p>Uses Newton-Raphson iteration to solve the full quartic time-of-flight equation
 * when the robot is moving, with a direct analytic fallback for stationary shots and
 * a distance-based estimate when convergence fails — ensuring power always varies
 * with distance.
 */
public class ProjectileSolver {

  public static class FiringSolution {
    /** Horizontal aim direction (degrees, 0 = +X, 90 = +Y). */
    public double yaw;
    /** Pitch angle above horizontal (degrees). */
    public double pitch;
    /** Required muzzle velocity magnitude (m/s). */
    public double power;
    /** Horizontal distance to target (m). */
    public double horizontalDistance;
    /** Whether a usable firing solution was found. */
    public boolean isValid;
    /** Projectile velocity in world frame (m/s). */
    public Translation3d worldVel;

    @Override
    public String toString() {
      return String.format("Valid: %b | Yaw: %.2f | Pitch: %.2f | Power: %.2f | Dist: %.2f",
          isValid, yaw, pitch, power, horizontalDistance);
    }
  }

  private static final double G = 9.81;
  /** Safety cap — unrealistic to need more. */
  private static final double MAX_POWER_MPS = 20.0;

  /**
   * Analytic muzzle velocity for a <em>stationary</em> shooter at the given pitch.
   * <pre>
   *   v² = g·d² / (2·cos²θ·(d·tanθ − Δz))
   * </pre>
   * Returns NaN when {@code d·tanθ ≤ Δz} (target below line of sight).
   */
  private static double stationaryVelocity(double d, double dz, double pitchRad) {
    double cos = Math.cos(pitchRad);
    double tan = Math.tan(pitchRad);
    double denom = d * tan - dz;
    if (denom <= 0.0) return Double.NaN;
    return Math.sqrt(G * d * d / (2.0 * cos * cos * denom));
  }

  /**
   * Core solve: computes muzzle velocity & time-of-flight for a moving/stationary shooter.
   */
  public static FiringSolution solve(
      Translation3d start,
      Translation3d target,
      Translation3d shooterVel,
      double muzzlePitchDegrees) {

    FiringSolution sol = new FiringSolution();
    Translation3d diff  = target.minus(start);
    double dx = diff.getX();
    double dy = diff.getY();
    double dz = diff.getZ();

    double vrx = shooterVel.getX();
    double vry = shooterVel.getY();
    double vrz = shooterVel.getZ();

    double d = Math.sqrt(dx * dx + dy * dy);
    sol.horizontalDistance = d;

    // --- Determine pitch ---
    double pitchRad = (Math.abs(muzzlePitchDegrees) < 0.1)
        ? Math.toRadians(45.0)
        : Math.toRadians(muzzlePitchDegrees);
    double sinTheta = Math.sin(pitchRad);
    double cosTheta = Math.cos(pitchRad);
    double tanTheta = Math.tan(pitchRad);
    double cotTheta = 1.0 / tanTheta;
    double cot2Theta = cotTheta * cotTheta;

    // --- Initial guess for time of flight (stationary approximation) ---
    double t;
    if (d * tanTheta > dz) {
      t = Math.sqrt(2.0 * (d * tanTheta - dz) / G);
    } else {
      t = 0.5;
    }

    // --- Quartic coefficients (moving shooter) ---
    // (dx - vrx·t)² + (dy - vry·t)² = cot²θ·(dz + ½g·t² − vrz·t)²
    double a = 0.5 * G;
    double b = -vrz;
    double c = dz;
    double k = cot2Theta;

    double A = k * (a * a);
    double B = k * (2.0 * a * b);
    double C = k * (b * b + 2.0 * a * c) - (vrx * vrx + vry * vry);
    double D = k * (2.0 * b * c) + 2.0 * (dx * vrx + dy * vry);
    double E = k * (c * c) - (dx * dx + dy * dy);

    // --- Newton-Raphson (up to 40 iterations, more tolerant) ---
    boolean converged = false;
    for (int i = 0; i < 40; i++) {
      double t2 = t * t;
      double t3 = t2 * t;
      double t4 = t3 * t;

      double f  = A * t4 + B * t3 + C * t2 + D * t + E;
      double fp = 4.0 * A * t3 + 3.0 * B * t2 + 2.0 * C * t + D;

      if (Math.abs(fp) < 1e-12) break;
      double delta = f / fp;
      t -= delta;
      if (t < 0.001) t = 0.001;
      if (Math.abs(delta) < 1e-8) { converged = true; break; }
    }

    // --- Compute muzzle velocity ---
    // Try the full moving-shooter solution; fall back to stationary analytic value.
    double vMuzzle;
    if (converged && t > 0.001 && !Double.isNaN(t)) {
      double vmz = (dz + 0.5 * G * t * t) / t - vrz;
      vMuzzle = vmz / sinTheta;
    } else {
      // Use analytic stationary formula — always works when d·tanθ > dz
      double vs = stationaryVelocity(d, dz, pitchRad);
      if (!Double.isNaN(vs) && vs <= MAX_POWER_MPS) {
        vMuzzle = vs;
        // Compute a crude time-of-flight from the stationary solution
        t = d / (vMuzzle * cosTheta);
      } else {
        // Even analytic failed — fall back to a distance-based estimate
        sol.isValid = false;
        sol.power   = estimateFallbackPower(d, dz, pitchRad);
        sol.pitch   = Math.toDegrees(pitchRad);
        sol.yaw     = Math.toDegrees(Math.atan2(dy, dx));
        sol.worldVel = new Translation3d();
        return sol;
      }
    }

    // --- Clamp to sane range ---
    if (vMuzzle > MAX_POWER_MPS) {
      // Best-effort: cap it and recalc time
      vMuzzle = MAX_POWER_MPS;
      sol.isValid = false;
    } else {
      sol.isValid = true;
    }

    // --- Fill output fields ---
    double vmx = dx / t - vrx;
    double vmy = dy / t - vry;
    double vmz = vMuzzle * sinTheta;

    sol.power = vMuzzle;
    sol.pitch = Math.toDegrees(pitchRad);
    sol.yaw   = Math.toDegrees(Math.atan2(vmy, vmx));
    sol.worldVel = new Translation3d(vmx + vrx, vmy + vry, vmz + vrz);
    return sol;
  }

  /**
   * Fallback that estimates power from horizontal distance alone.
   * Ensures the value is always > 0 and varies with distance.
   */
  private static double estimateFallbackPower(double d, double dz, double pitchRad) {
    double vs = stationaryVelocity(d, dz, pitchRad);
    if (!Double.isNaN(vs)) return Math.max(vs, 6.0);
    // Target too close — use a linear ramp starting at 6 m/s
    return Math.max(6.0, 5.0 + d * 1.5);
  }
}
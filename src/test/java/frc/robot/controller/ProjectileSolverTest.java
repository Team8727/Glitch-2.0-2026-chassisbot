package frc.robot.controller;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class ProjectileSolverTest {
  // This must match Robot.SHOOTER_ANGLE_DEGREES so the test uses the same angle as the robot.
  private static final double TEST_ANGLE_DEG = 45.0;

  @Test
  void solveProducesExpectedSolutionWhenReachable() {
    Translation3d start = new Translation3d(0.0, 0.0, 0.0);
    Translation3d target = new Translation3d(5.0, 0.0, 1.0);
    Translation3d shooterVel = new Translation3d();

    ProjectileSolver.FiringSolution sol = ProjectileSolver.solve(start, target, shooterVel, TEST_ANGLE_DEG);

    assertTrue(sol.isValid);
    assertEquals(0.0, sol.yaw, 1e-3);
    assertEquals(TEST_ANGLE_DEG, sol.pitch, 0.1);
    // At 45°, 5 m horizontal, 1 m height diff: muzzle velocity ≈ 7.83 m/s
    assertEquals(7.83, sol.power, 0.05);
    assertEquals(5.0, sol.horizontalDistance, 1e-3);
  }

  @Test
  void solveYawMatchesGroundDirection() {
    Translation3d start = new Translation3d(0.0, 0.0, 0.0);
    Translation3d target = new Translation3d(0.0, 5.0, 1.0);

    ProjectileSolver.FiringSolution sol = ProjectileSolver.solve(start, target, new Translation3d(), TEST_ANGLE_DEG);

    assertTrue(sol.isValid);
    assertEquals(90.0, sol.yaw, 1e-2);
    assertEquals(5.0, sol.horizontalDistance, 1e-3);
  }

  @Test
  void solveMarksInvalidWhenUnreachable() {
    Translation3d start = new Translation3d(0.0, 0.0, 0.0);
    Translation3d target = new Translation3d(1.0, 0.0, 10.0);

    ProjectileSolver.FiringSolution sol = ProjectileSolver.solve(start, target, new Translation3d(), TEST_ANGLE_DEG);

    assertFalse(sol.isValid);
    assertEquals(1.0, sol.horizontalDistance, 1e-3);
  }

  @Test
  void shooterVelocityCompensationReducesRequiredPower() {
    Translation3d start = new Translation3d(0.0, 0.0, 0.0);
    Translation3d target = new Translation3d(5.0, 0.0, 1.0);

    ProjectileSolver.FiringSolution stationary = ProjectileSolver.solve(start, target, new Translation3d(), TEST_ANGLE_DEG);
    ProjectileSolver.FiringSolution moving = ProjectileSolver.solve(start, target, new Translation3d(2.0, 0.0, 0.0), TEST_ANGLE_DEG);

    assertTrue(stationary.isValid);
    assertTrue(moving.isValid);
    assertEquals(0.0, moving.yaw, 1e-3);
    assertEquals(5.0, stationary.horizontalDistance, 1e-3);
    assertEquals(5.0, moving.horizontalDistance, 1e-3);
    assertTrue(moving.power < stationary.power);
  }
}


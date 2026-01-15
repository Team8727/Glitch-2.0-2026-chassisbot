package frc.robot.controller;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class ProjectileSolverTest {

  @Test
  void solveProducesExpectedSolutionWhenReachable() {
    Translation3d start = new Translation3d(0.0, 0.0, 0.0);
    Translation3d target = new Translation3d(5.0, 0.0, 1.0);
    Translation3d shooterVel = new Translation3d();

    ProjectileSolver.FiringSolution sol = ProjectileSolver.solve(start, target, shooterVel, 0.0);

    assertTrue(sol.isValid);
    assertEquals(0.0, sol.yaw, 1e-3);
    assertEquals(158.2, sol.pitch, 0.1);
    assertEquals(11.94, sol.power, 0.05);
  }

  @Test
  void solveYawMatchesGroundDirection() {
    Translation3d start = new Translation3d(0.0, 0.0, 0.0);
    Translation3d target = new Translation3d(0.0, 5.0, 1.0);

    ProjectileSolver.FiringSolution sol = ProjectileSolver.solve(start, target, new Translation3d(), 0.0);

    assertTrue(sol.isValid);
    assertEquals(90.0, sol.yaw, 1e-2);
  }

  @Test
  void solveMarksInvalidWhenDenominatorNonPositive() {
    Translation3d start = new Translation3d(0.0, 0.0, 0.0);
    Translation3d target = new Translation3d(5.0, 0.0, 0.0);

    ProjectileSolver.FiringSolution sol = ProjectileSolver.solve(start, target, new Translation3d(), 45.0);

    assertFalse(sol.isValid);
  }

  @Test
  void shooterVelocityCompensationReducesRequiredPower() {
    Translation3d start = new Translation3d(0.0, 0.0, 0.0);
    Translation3d target = new Translation3d(5.0, 0.0, 1.0);

    ProjectileSolver.FiringSolution stationary = ProjectileSolver.solve(start, target, new Translation3d(), 0.0);
    ProjectileSolver.FiringSolution moving = ProjectileSolver.solve(start, target, new Translation3d(2.0, 0.0, 0.0), 0.0);

    assertTrue(stationary.isValid);
    assertTrue(moving.isValid);
    assertEquals(0.0, moving.yaw, 1e-3);
    assertTrue(moving.power < stationary.power);
  }
}


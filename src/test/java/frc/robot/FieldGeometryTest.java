// Copyright (c) FRC Team 8727
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Verifies the field-geometry constants are in the correct coordinate frame.
 *
 * <p>The target coordinates were historically expressed in a <em>corner-origin</em> frame
 * (origin on the BLUE alliance wall, +X toward RED), while the drivetrain pose always uses
 * the WPILib <em>center-origin</em> frame (origin at field center, +X toward RED). Mixing
 * the two frames inflated the computed shooter range ~3x and saturated the flywheel.
 *
 * <p>These tests lock in the center-origin fix and guard against re-introducing the
 * corner-origin values (4.626 / 11.915).
 */
class FieldGeometryTest {

  /** Valid tolerances account for the computed constant derivation. */
  private static final double EPSILON = 0.01;

  @Test
  void blueTargetIsInCenterOriginFrame() {
    Translation3d t = Robot.getBlueTarget();

    // Blue HUB is ~4.24 m in from the blue wall, which sits at -8.27 in center-origin.
    // So the HUB center X is -8.27 + 4.03 = -4.24. The old value was +4.626 in corner-origin.
    assertEquals(-4.24, t.getX(), EPSILON,
            "Blue HUB X must be -4.24 m in the center-origin WPILib frame");

    // Both HUBs are centered between two BUMPS, so Y is the field centerline.
    assertEquals(0.0, t.getY(), EPSILON,
            "HUB Y must be on the field centerline (between BUMPS / §5.4)");

    // HUB opening front edge is 72 in = 1.83 m off the carpet (manual line 469).
    assertEquals(1.83, t.getZ(), EPSILON,
            "HUB Z must be 1.83 m (72 in) — manual §5.4, line 469");
  }

  @Test
  void redTargetIsInCenterOriginFrame() {
    Translation3d t = Robot.getRedTarget();

    // Red HUB is ~4.24 m in from the red wall, which sits at +8.27 in center-origin.
    // So the HUB center X is 8.27 - 4.03 = +4.24. The old value was 11.915 in corner-origin.
    assertEquals(4.24, t.getX(), EPSILON,
            "Red HUB X must be +4.24 m in the center-origin WPILib frame");

    assertEquals(0.0, t.getY(), EPSILON, "HUB Y must be on the field centerline");
    assertEquals(1.83, t.getZ(), EPSILON, "HUB Z must be 1.83 m (72 in)");
  }

  @Test
  void hubsAreSymmetricAcrossFieldCenter() {
    double blueX = Robot.getBlueTarget().getX();
    double redX  = Robot.getRedTarget().getX();

    // In a center-origin frame the two HUBs sit symmetrically around X = 0.
    assertEquals(0.0, blueX + redX, EPSILON,
            "Blue and Red X should sum to zero (field center origin)");
  }

  @Test
  void targetsAreConstantAndPrecomputed() {
    // The computed constants should never change unless the field geometry changes.
    // If this test fails after a deliberate field-model update, update the expected
    // values AND the manual citations in Robot.java.
    assertEquals(-4.24, Robot.getBlueTarget().getX(), EPSILON);
    assertEquals( 4.24, Robot.getRedTarget().getX(),  EPSILON);
    assertEquals( 0.00, Robot.getBlueTarget().getY(), EPSILON);
    assertEquals( 0.00, Robot.getRedTarget().getY(),  EPSILON);
    assertEquals( 1.83, Robot.getBlueTarget().getZ(), EPSILON);
    assertEquals( 1.83, Robot.getRedTarget().getZ(),  EPSILON);
  }

  @Test
  void oldCornerOriginValuesAreRejected() {
    // 4.626 and 11.915 were the 2020 Infinite Recharge constants in corner-origin.
    // They sum to 16.541 which is the field length, confirming they were in the
    // wrong frame. Verify they have been replaced.
    assertNotEquals(4.626, Robot.getBlueTarget().getX(), EPSILON,
            "Old corner-origin value 4.626 must be gone");
    assertNotEquals(11.915, Robot.getRedTarget().getX(), EPSILON,
            "Old corner-origin value 11.915 must be gone");
  }
}
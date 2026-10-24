package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Central constants for the 2026 REBUILT robot — Team 8727 Glitch 2.0.
 *
 * <p>Hardware IDs, field geometry, shooter parameters, tuning values, and
 * mechanism limits are defined here so every subsystem reads from one source.
 */
public final class Constants {

  private Constants() {}

  // ========== CAN Bus ==========

  /** Primary swerve drive CAN bus name. */
  public static final String kCANBusName = "Swerve Drive";

  // ========== Shooter (4× NEO SparkMax) ==========

  public static final int kShooterLeaderId = 6;
  public static final int kShooterFollower1Id = 7;
  public static final int kShooterFollower2Id = 8;
  public static final int kShooterFollower3Id = 9;

  public static final int kShooterCurrentLimit = 60;

  /** Flywheel PID kP (from SysID). */
  public static final double kShooterKP = 0.07;

  /** Feedforward gains (SysID-identified). */
  public static final double kShooterKS = 0.35091;
  public static final double kShooterKV = 0.12701;
  public static final double kShooterKA = 0.052063;

  /** Encoder conversion factor for RPS output. */
  public static final double kShooterEncoderConversion = 1.0 / 60.0;

  /** Gear ratio between motor and flywheel (output/input). 24:15 = 1.6 (motor faster). */
  public static final double kFlywheelGearing = 24.0 / 15.0;

  /** Flywheel moment of inertia (kg·m²). */
  public static final double kFlywheelMomentOfInertia = 0.0012112355;

  /** State-space model variance: how accurate we think our model is. */
  public static final double kShooterModelStdDev = 3.0;
  /** State-space encoder measurement std dev. */
  public static final double kShooterEncoderStdDev = 0.01;
  /** LQR velocity error tolerance (rad/s). */
  public static final double kShooterQelms = 8.0;
  /** LQR control effort tolerance (V). */
  public static final double kShooterRelms = 8.0;

  // ========== Intake (1× NEO SparkMax) ==========

  public static final int kIntakeId = 3;
  public static final int kIntakeCurrentLimit = 60;

  // ========== Indexer (2× NEO SparkMax) ==========

  public static final int kIndexerLeaderId = 4;
  public static final int kIndexerFollowerId = 5;
  public static final int kIndexerLeaderCurrentLimit = 30;
  public static final int kIndexerFollowerCurrentLimit = 60;

  // ========== LEDs ==========

  public static final int kCANdleId = 49;

  public static final int kLEDTotal = 88;
  public static final int kLEDLeft = 40;
  public static final int kLEDPip = 8;
  public static final int kLEDRight = 40;

  // ========== Vision ==========

  public static final String kCamFrontLeft = "FrontLeft";
  public static final String kCamFrontRight = "FrontRight";

  /** Camera transforms relative to robot center (meters, radians). */
  public static final Translation3d kCamFrontLeftTranslation =
      new Translation3d(
          Units.inchesToMeters(2.375),
          Units.inchesToMeters(3.25),
          Units.inchesToMeters(28));
  public static final double kCamFrontLeftYaw = Math.toRadians(10);

  public static final Translation3d kCamFrontRightTranslation =
      new Translation3d(
          Units.inchesToMeters(2.375),
          Units.inchesToMeters(-2.875),
          Units.inchesToMeters(28));
  public static final double kCamFrontRightYaw = Math.toRadians(-10);

  public static final double kVisionMaxAmbiguity = 0.2;
  public static final double kVisionMaxDistanceMeters = 3.5;

  // ========== Field Geometry (2026 REBUILT, corner-origin frame) ==========
  // These match the PathPlanner .path files, the AprilTag field layout JSON,
  // and the chassisbot codebase (which shoots correctly).
  //
  // Corner-origin: (0,0) = blue alliance wall / left long wall.
  // X increases toward red alliance wall.  Y increases across the field width.
  // field-length = 16.518 m, field-width = 8.043 m
  // Blue HUB tags (IDs 25, 26) are at x ≈ 4.008, y ≈ 4.021 (image coords).

  public static final double kFieldLengthMeters = 16.518;
  public static final double kFieldWidthMeters = 8.043;

  /** HUB note-scoring center height above carpet. */
  public static final double kHubZMeters = 1.83;

  /** Blue HUB target — from chassisbot field-tested values. */
  public static final Translation3d kBlueHubTarget =
      new Translation3d(4.626, 4.035, kHubZMeters);
  /** Red HUB target. */
  public static final Translation3d kRedHubTarget =
      new Translation3d(11.915, 4.035, kHubZMeters);

  // ========================================================================
  // Passing & aim positions (selected by the right bumper) — corner-origin
  // ========================================================================

  /** Neutral-zone passing target (Blue). */
  public static final Translation3d kBlueNeutralPass =
      new Translation3d(8.259, 5.235, kHubZMeters);  // field center + slight Y offset
  /** Neutral-zone passing target (Red). */
  public static final Translation3d kRedNeutralPass =
      new Translation3d(8.259, 2.835, kHubZMeters);  // field center, opposite strafe

  /** Alliance-area target (Blue) — short-range pass. */
  public static final Translation3d kBlueAllianceTarget =
      new Translation3d(5.759, 4.835, kHubZMeters);
  /** Alliance-area target (Red). */
  public static final Translation3d kRedAllianceTarget =
      new Translation3d(10.759, 3.235, kHubZMeters);

  // ========== Shooter Physical ==========

  /** Fixed shooter pitch (degrees above horizontal). */
  public static final double kShooterAngleDegrees = 60.0;

  /** Shooter centerline height above carpet (m). */
  public static final double kShooterHeightMeters = 0.3;

  /** Flywheel diameter (m). */
  public static final double kShooterFlywheelDiameterMeters = 0.0889;

  // ========== Scoring / Shooting ==========

  /** Time to spin up flywheel before feeding (s). */
  public static final double kShooterSpinupTime = 2.0;

  /** Time to feed indexer while shooting (s). */
  public static final double kShooterFeedTime = 1.0;

  /** Indexer duty cycle during shooting. */
  public static final double kShooterIndexerDutyCycle = 0.8;

  /** Intake duty cycle for normal operation. */
  public static final double kIntakeDutyCycle = 0.8;

  /** Indexer duty cycle forward. */
  public static final double kIndexerForwardDutyCycle = 0.8;

  /** Indexer duty cycle reverse (unjam). */
  public static final double kIndexerReverseDutyCycle = -1.0;

  // ========== Auto Max Speeds (PathPlanner) ==========

  public static final double kAutoMaxLinearVelocity = 4.39;
  public static final double kAutoMaxLinearAcceleration = 2.0;
  public static final double kAutoMaxAngularVelocity = 0.75 * 2.0 * Math.PI;
  public static final double kAutoMaxAngularAcceleration = 0.4 * 3.0 * Math.PI;

  // ========== Teleop Drive Tuning ==========

  /** Max teleop translation speed as fraction of max. */
  public static final double kTeleopSpeedFraction = 0.8;

  /** Rotation PID gains for FieldCentricFacingAngle. */
  public static final double kRotationKP = 25.0;
  public static final double kRotationKDSim = 0.15;
  public static final double kRotationKDReal = 0.15;

  // ========== Vision Pose Fusion ==========

  /** Distance-scaled covariance baseline (Lynk-style). */
  public static final double kVisionLinearStdDevBaseline = 0.10;
  public static final double kVisionAngularStdDevBaseline = 0.35;
  public static final double kVisionUnknownDistanceMeters = 2.0;
}
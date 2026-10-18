package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;

import java.util.HashMap;
import java.util.Map;

/**
 * Lightweight NetworkTables logger with cached publishers.
 *
 * <p>Publishers are created once and reused, avoiding the NT4 limit on
 * concurrent publisher objects.
 */
public class NetworkTableLogger {
  private final String root;

  private final Map<String, DoublePublisher> doublePubs = new HashMap<>();
  private final Map<String, IntegerPublisher> intPubs = new HashMap<>();
  private final Map<String, BooleanPublisher> boolPubs = new HashMap<>();
  private final Map<String, StringPublisher> stringPubs = new HashMap<>();
  private final Map<String, StructPublisher<Pose2d>> pose2dPubs = new HashMap<>();
  private final Map<String, StructPublisher<Pose3d>> pose3dPubs = new HashMap<>();
  private final Map<String, StructPublisher<ChassisSpeeds>> chassisPubs = new HashMap<>();
  private final Map<String, StructArrayPublisher<SwerveModuleState>> moduleStatePubs = new HashMap<>();
  private final Map<String, StructPublisher<Translation3d>> translationPubs = new HashMap<>();

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  public NetworkTableLogger(String root) {
    this.root = root;
  }

  private String key(String k) { return root + "/" + k; }

  public void logDouble(String key, double value) {
    doublePubs.computeIfAbsent(key(key), k -> inst.getDoubleTopic(k).publish()).set(value);
  }

  public void logInt(String key, int value) {
    intPubs.computeIfAbsent(key(key), k -> inst.getIntegerTopic(k).publish()).set(value);
  }

  public void logBoolean(String key, boolean value) {
    boolPubs.computeIfAbsent(key(key), k -> inst.getBooleanTopic(k).publish()).set(value);
  }

  public void logString(String key, String value) {
    stringPubs.computeIfAbsent(key(key), k -> inst.getStringTopic(k).publish()).set(value);
  }

  public void logPose2d(String key, Pose2d value) {
    pose2dPubs.computeIfAbsent(key(key), k -> inst.getStructTopic(k, Pose2d.struct).publish()).set(value);
  }

  public void logPose3d(String key, Pose3d value) {
    pose3dPubs.computeIfAbsent(key(key), k -> inst.getStructTopic(k, Pose3d.struct).publish()).set(value);
  }

  public void logChassisSpeeds(String key, ChassisSpeeds value) {
    chassisPubs.computeIfAbsent(key(key), k -> inst.getStructTopic(k, ChassisSpeeds.struct).publish()).set(value);
  }

  public void logSwerveModuleStates(String key, SwerveModuleState[] value) {
    moduleStatePubs.computeIfAbsent(key(key), k -> inst.getStructArrayTopic(k, SwerveModuleState.struct).publish()).set(value);
  }

  public void logTranslation3d(String key, Translation3d value) {
    translationPubs.computeIfAbsent(key(key), k -> inst.getStructTopic(k, Translation3d.struct).publish()).set(value);
  }
}
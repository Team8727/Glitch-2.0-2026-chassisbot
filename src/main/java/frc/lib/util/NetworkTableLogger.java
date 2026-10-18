package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;

/**
 * Lightweight lazy-initialized NetworkTables logger.
 * Publishes typed values under a configurable root table.
 */
public class NetworkTableLogger {
  private final String root;

  public NetworkTableLogger(String root) {
    this.root = root;
  }

  private String key(String k) { return root + "/" + k; }

  // ---- Lazily-created publishers ----

  public void logDouble(String key, double value) {
    var topic = NetworkTableInstance.getDefault().getDoubleTopic(key(key));
    var pub = topic.publish();
    pub.set(value);
  }

  public void logInt(String key, int value) {
    var topic = NetworkTableInstance.getDefault().getIntegerTopic(key(key));
    var pub = topic.publish();
    pub.set(value);
  }

  public void logBoolean(String key, boolean value) {
    var topic = NetworkTableInstance.getDefault().getBooleanTopic(key(key));
    var pub = topic.publish();
    pub.set(value);
  }

  public void logString(String key, String value) {
    var topic = NetworkTableInstance.getDefault().getStringTopic(key(key));
    var pub = topic.publish();
    pub.set(value);
  }

  public void logPose2d(String key, Pose2d value) {
    var topic = NetworkTableInstance.getDefault().getStructTopic(key(key), Pose2d.struct);
    var pub = topic.publish();
    pub.set(value);
  }

  public void logPose3d(String key, Pose3d value) {
    var topic = NetworkTableInstance.getDefault().getStructTopic(key(key), Pose3d.struct);
    var pub = topic.publish();
    pub.set(value);
  }

  public void logChassisSpeeds(String key, ChassisSpeeds value) {
    var topic = NetworkTableInstance.getDefault().getStructTopic(key(key), ChassisSpeeds.struct);
    var pub = topic.publish();
    pub.set(value);
  }

  public void logSwerveModuleStates(String key, SwerveModuleState[] value) {
    var topic = NetworkTableInstance.getDefault().getStructArrayTopic(key(key), SwerveModuleState.struct);
    var pub = topic.publish();
    pub.set(value);
  }

  public void logTranslation3d(String key, Translation3d value) {
    var topic = NetworkTableInstance.getDefault().getStructTopic(key(key), Translation3d.struct);
    var pub = topic.publish();
    pub.set(value);
  }
}
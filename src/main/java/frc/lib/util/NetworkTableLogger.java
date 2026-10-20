package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;

import java.util.HashMap;
import java.util.Map;

/**
 * Simple NetworkTables publisher with cached publishers.
 * Creates each publisher once, reuses it forever. Avoids NT4's publisher limit.
 */
public class NetworkTableLogger {
  private final String root;
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private final Map<String, DoublePublisher>      dbl = new HashMap<>();
  private final Map<String, IntegerPublisher>     int_ = new HashMap<>();
  private final Map<String, BooleanPublisher>     bool = new HashMap<>();
  private final Map<String, StringPublisher>      str = new HashMap<>();
  private final Map<String, StructPublisher<Pose3d>> p3 = new HashMap<>();

  public NetworkTableLogger(String root) { this.root = root; }
  private String n(String k) { return root + "/" + k; }

  public void logDouble(String key, double v) {
    String t = n(key);
    if (!dbl.containsKey(t)) dbl.put(t, inst.getDoubleTopic(t).publish());
    dbl.get(t).set(v);
  }

  public void logInt(String key, int v) {
    String t = n(key);
    if (!int_.containsKey(t)) int_.put(t, inst.getIntegerTopic(t).publish());
    int_.get(t).set(v);
  }

  public void logBoolean(String key, boolean v) {
    String t = n(key);
    if (!bool.containsKey(t)) bool.put(t, inst.getBooleanTopic(t).publish());
    bool.get(t).set(v);
  }

  public void logString(String key, String v) {
    String t = n(key);
    if (!str.containsKey(t)) str.put(t, inst.getStringTopic(t).publish());
    str.get(t).set(v);
  }

  public void logPose3d(String key, Pose3d v) {
    String t = n(key);
    if (!p3.containsKey(t)) p3.put(t, inst.getStructTopic(t, Pose3d.struct).publish());
    p3.get(t).set(v);
  }
}
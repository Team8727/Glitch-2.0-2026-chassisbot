package frc.lib.util;

import edu.wpi.first.networktables.*;

import java.util.HashMap;
import java.util.Map;

/**
 * Cached NT4 reader for reading double/string values published by the dashboard.
 * Mirrors {@link NetworkTableLogger} but for INPUT from dashboard controls (sliders, choosers).
 *
 * <p>Uses {@link NetworkTableEntry} internally for backwards-compatible NT3/NT4 access.
 */
public class NetworkTableReader {
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table;
  private final Map<String, NetworkTableEntry> doubleEntries = new HashMap<>();
  private final Map<String, NetworkTableEntry> stringEntries = new HashMap<>();

  public NetworkTableReader(String root) {
    this.table = inst.getTable(root.startsWith("/") ? root.substring(1) : root);
  }

  public double getDouble(String key, double defaultValue) {
    if (!doubleEntries.containsKey(key)) {
      doubleEntries.put(key, table.getEntry(key));
    }
    return doubleEntries.get(key).getDouble(defaultValue);
  }

  public String getString(String key, String defaultValue) {
    if (!stringEntries.containsKey(key)) {
      stringEntries.put(key, table.getEntry(key));
    }
    return stringEntries.get(key).getString(defaultValue);
  }
}
package frc.robot.config;

import static java.util.Map.entry;

import edu.wpi.first.wpilibj.Preferences;
import java.util.Map;
import java.util.logging.Logger;

/**
 * The DynamicRobotConfig class keeps track of periodically changing constants on the robot
 *
 * <p>This should only have offsets that would change with a mechanical change. There should be no
 * IDs, Conversions, or setpoint information in this config.
 */
public final class DynamicRobotConfig {

  private static boolean initialized = false;

  // Keys to load values from, shall not be modified under any circumstance
  private static final String frontLeftOffset_key = "front Left Offset";
  private static final String frontRightOffset_key = "front Right Offset";
  private static final String backRightOffset_key = "back Right Offset";
  private static final String backLeftOffset_key = "back Left Offset";

  // Contains every key that is used
  // if one is missing it won't be correctly saved, or init
  private static final String[] allKeys = {
    frontLeftOffset_key, frontRightOffset_key, backRightOffset_key, backLeftOffset_key
  };

  private static Map<String, Double> dblMap =
      Map.ofEntries(
          entry(frontLeftOffset_key, 0.0),
          entry(frontRightOffset_key, 0.0),
          entry(backLeftOffset_key, 0.0),
          entry(backRightOffset_key, 0.0));

  private static Map<String, Integer> intMap = Map.ofEntries();
  private static Map<String, String> strMap = Map.ofEntries();
  private static Map<String, Boolean> boolMap = Map.ofEntries();

  public static final double frontLeftOffset() {
    return lookupDbl(frontLeftOffset_key);
  }

  public static final double frontRightOffset() {
    return lookupDbl(frontRightOffset_key);
  }

  public static final double backRightOffset() {
    return lookupDbl(backRightOffset_key);
  }

  public static final double backLeftOffset() {
    return lookupDbl(backLeftOffset_key);
  }

  private static double lookupDbl(String key) {
    if (!initialized) {
      init();
    }
    return dblMap.get(key);
  }

  public static TunerConstants getTunerConstants() {
    return new TunerConstants(
        frontLeftOffset(), frontRightOffset(), backLeftOffset(), backRightOffset());
  }

  private static void init() {

    boolean initNT = false;

    for (String key : dblMap.keySet()) {
      if (Preferences.containsKey(key)) {
        dblMap.put(key, Preferences.getDouble(key, 0d));
      } else {
        raiseWarning(key + " NOT FOUND!! using default");
        initNT = true;
      }
    }

    for (String key : intMap.keySet()) {
      if (Preferences.containsKey(key)) {
        intMap.put(key, Preferences.getInt(key, 0));
      } else {
        raiseWarning(key + " NOT FOUND!! using default");
        initNT = true;
      }
    }

    for (String key : strMap.keySet()) {
      if (Preferences.containsKey(key)) {
        strMap.put(key, Preferences.getString(key, ""));
      } else {
        raiseWarning(key + " NOT FOUND!! using default");
        initNT = true;
      }
    }

    for (String key : boolMap.keySet()) {
      if (Preferences.containsKey(key)) {
        boolMap.put(key, Preferences.getBoolean(key, false));
      } else {
        raiseWarning(key + " NOT FOUND!! using default");
        initNT = true;
      }
    }

    if (initNT) {
      initNT();
    }
    initialized = true;
  }

  /** Creates network table entries even if not pre-existing */
  private static void initNT() {
    logInfo("Initlizing Dynamic Logs");
    for (String key : allKeys) {
      Preferences.initDouble(key, 0);
    }
  }

  public static void loadDynamicRobotConfig() {
    if (!initialized) {
      init();
    }
  }

  private static Logger DRC_logger = Logger.getLogger(DynamicRobotConfig.class.getName());

  private static void raiseWarning(String warning) {
    DRC_logger.warning(warning);
  }

  private static void logInfo(String info) {
    DRC_logger.info(info);
  }
}

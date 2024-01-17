package frc.robot.config;

import static java.util.Map.entry;

import edu.wpi.first.wpilibj.Preferences;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
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

  private static Map<String, PreferenceObject> prefMap =
      Map.ofEntries(
          entry(frontLeftOffset_key, new PreferenceObject(Double.class)),
          entry(frontRightOffset_key, new PreferenceObject(Double.class)),
          entry(backLeftOffset_key, new PreferenceObject(Double.class)),
          entry(backRightOffset_key, new PreferenceObject(Double.class)));

  // Contains every key that is used
  // if one is missing it won't be correctly saved, or init
  private static final String[] allKeys = {
    frontLeftOffset_key, frontRightOffset_key, backRightOffset_key, backLeftOffset_key
  };

  public static final double frontLeftOffset() {
    return (double) lookup(frontLeftOffset_key);
  }

  public static final double frontRightOffset() {
    return (double) lookup(frontRightOffset_key);
  }

  public static final double backRightOffset() {
    return (double) lookup(backRightOffset_key);
  }

  public static final double backLeftOffset() {
    return (double) lookup(backLeftOffset_key);
  }

  private static class PreferenceObject {
    public Object data;
    public Function<String, Object> loadPreference;
    public Consumer<String> initPreference;

    public PreferenceObject(Class<?> c) {
      // spotless:off
      if (c == Double.class) {
        loadPreference = (a) -> { return Preferences.getDouble(a, 0d); };
        initPreference = (a) -> {Preferences.initDouble(a, 0d);};
      } else if (c == Integer.class) {
        loadPreference = (a) -> { return Preferences.getInt(a, 0); };
        initPreference = (a) -> {Preferences.initInt(a, 0);};
      } else if (c == String.class) {
        loadPreference = (a) -> { return Preferences.getString(a, ""); };
        initPreference = (a) -> {Preferences.initString(a, "");};
      } else if (c == Boolean.class) {
        loadPreference = (a) -> { return Preferences.getBoolean(a, false); };
        initPreference = (a) -> {Preferences.initBoolean(a, false);};
      }
      // spotless:on
    }
  }

  private static Object lookup(String key) {
    if (!initialized) {
      init();
    }
    return prefMap.get(key).data;
  }

  public static TunerConstants getTunerConstants() {
    return new TunerConstants(
        frontLeftOffset(), frontRightOffset(), backLeftOffset(), backRightOffset());
  }

  private static void init() {

    boolean initNT = false;

    for (String key : prefMap.keySet()) {
      if (Preferences.containsKey(key)) {
        PreferenceObject pref = prefMap.get(key);
        pref.data = pref.loadPreference.apply(key);
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
      PreferenceObject pref = prefMap.get(key);
      pref.initPreference.accept(key);
      pref.data = pref.loadPreference.apply(key);
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

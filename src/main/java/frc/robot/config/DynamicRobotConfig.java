package frc.robot.config;

import edu.wpi.first.wpilibj.Preferences;
import java.util.logging.Logger;

/**
 * The DynamicRobotConfig class keeps track of periodically changing constants on the robot
 *
 * <p>This should only have offsets that would change with a mechanical change. There should be no
 * IDs, Conversions, or setpoint information in this config.
 */
public class DynamicRobotConfig {
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

  public final double frontLeftOffset;
  public final double frontRightOffset;
  public final double backRightOffset;
  public final double backLeftOffset;

  private DynamicRobotConfig() {
    if (Preferences.containsKey(frontLeftOffset_key)) {
      frontLeftOffset = Preferences.getDouble(frontLeftOffset_key, 0d);
    } else {
      raiseWarning("Front Left Pod Offset NOT FOUND!! using default");
      frontLeftOffset = 0;
    }

    if (Preferences.containsKey(frontRightOffset_key)) {
      frontRightOffset = Preferences.getDouble(frontRightOffset_key, 0d);
    } else {
      raiseWarning("Front Right Pod Offset NOT FOUND!! using default");
      frontRightOffset = 0;
    }

    if (Preferences.containsKey(backRightOffset_key)) {
      backRightOffset = Preferences.getDouble(backRightOffset_key, 0d);
    } else {
      raiseWarning("Back Right Pod Offset NOT FOUND!! using default");
      backRightOffset = 0;
    }

    if (Preferences.containsKey(backLeftOffset_key)) {
      backLeftOffset = Preferences.getDouble(backLeftOffset_key, 0d);
    } else {
      raiseWarning("Back Left Pod Offset NOT FOUND!! using default");
      backLeftOffset = 0;
    }
  }

  public static void save() {

    // If any NT is missing stop saving
    String missing;
    if ((missing = verifyPrefenceExistent(allKeys)).isEmpty()) {
      raiseWarning("Missing Entry:"+missing);
      return;
    }
  }

  /** Creates network table entries even if not pre-existing */
  public static void initNT() {}

  /**
   * Verifies that prefences all the given prefences exist
   *
   * @param keys - The keys to verify that they exist
   * @return the missing key, empty if all are present
   */
  private static String verifyPrefenceExistent(String[] keys) {
    for (String key : keys) {
      if (!Preferences.containsKey(key)) {
        return key;
      }
    }
    return "";
  }

  public static DynamicRobotConfig loadDynamicRobotConfig() {
    return new DynamicRobotConfig();
  }

  private static Logger DRC_logger = Logger.getLogger(DynamicRobotConfig.class.getName());

  private static void raiseWarning(String warning) {
    DRC_logger.warning(warning);
  }
}

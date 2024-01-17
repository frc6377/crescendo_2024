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
  private static DynamicRobotConfig dynRobotConfig;

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

  public TunerConstants getTunerConstants() {
    return new TunerConstants(frontLeftOffset, frontRightOffset, backLeftOffset, backRightOffset);
  }

  private DynamicRobotConfig() {
    boolean initNT = false;

    if (Preferences.containsKey(frontLeftOffset_key)) {
      frontLeftOffset = Preferences.getDouble(frontLeftOffset_key, 0d);
    } else {
      raiseWarning("Front Left Pod Offset NOT FOUND!! using default");
      frontLeftOffset = 0;
      initNT = true;
    }

    if (Preferences.containsKey(frontRightOffset_key)) {
      frontRightOffset = Preferences.getDouble(frontRightOffset_key, 0d);
    } else {
      raiseWarning("Front Right Pod Offset NOT FOUND!! using default");
      frontRightOffset = 0;
      initNT = true;
    }

    if (Preferences.containsKey(backRightOffset_key)) {
      backRightOffset = Preferences.getDouble(backRightOffset_key, 0d);
    } else {
      raiseWarning("Back Right Pod Offset NOT FOUND!! using default");
      backRightOffset = 0;
      initNT = true;
    }

    if (Preferences.containsKey(backLeftOffset_key)) {
      backLeftOffset = Preferences.getDouble(backLeftOffset_key, 0d);
    } else {
      raiseWarning("Back Left Pod Offset NOT FOUND!! using default");
      backLeftOffset = 0;
      initNT = true;
    }

    if (initNT) {
      initNT();
    }
  }

  /** Creates network table entries even if not pre-existing */
  public static void initNT() {
    logInfo("Initializing Dynamic Logs");
    for (String key : allKeys) {
      Preferences.initDouble(key, 0);
    }
  }

  public static DynamicRobotConfig loadDynamicRobotConfig() {
    if (dynRobotConfig != null) {
      return dynRobotConfig;
    }
    return new DynamicRobotConfig();
  }

  private static Logger DRC_logger = Logger.getLogger(DynamicRobotConfig.class.getName());

  private static void raiseWarning(String warning) {
    DRC_logger.warning(warning);
  }

  private static void logInfo(String info) {
    DRC_logger.info(info);
  }
}

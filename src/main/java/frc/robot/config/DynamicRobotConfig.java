package frc.robot.config;

import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Robot;
import java.lang.reflect.Field;
import java.util.logging.Logger;

/**
 * The DynamicRobotConfig class keeps track of periodically changing constants on the robot
 *
 * <p>This should only have offsets that would change with a mechanical change. There should be no
 * IDs, Conversions, or setpoint information in this config.
 */
public class DynamicRobotConfig {
  private static class ConfigVariables {
    public static double frontLeftOffset = 0;
    public static double frontRightOffset = 0;
    public static double backLeftOffset = 2;
    public static double backRightOffset = 0;
  }

  private static Logger logger = Logger.getLogger(DynamicRobotConfig.class.getName());

  private DynamicRobotConfig() {
    // This code interacts with ConfigVariables using reflection, essentially using it as a map.
    // First, the code gets a reference to every variable from the static class, referred to as a
    // "Field"
    // It then iterates through this list of references
    for (Field variable : ConfigVariables.class.getFields()) {
      // For each field, its name is stored as a string, so it can be used as a preference key
      String key = variable.getName();
      try {
        // If the preference does not exist, or we are bypassing to competition robot defaults, log
        // an alert and open network access for that preference
        if (!Preferences.containsKey(key) || Robot.isCompetition) {
          raiseWarning("Using competition robot default for " + key);
          Preferences.initDouble(key, variable.getDouble(ConfigVariables.class));
        }
        // Set the field to the value pulled from preferences. If the preference encounters another
        // error, use the field's startup value.
        else {
          variable.setDouble(
              ConfigVariables.class,
              Preferences.getDouble(key, variable.getDouble(ConfigVariables.class)));
        }
      } catch (IllegalAccessException e) {
        raiseWarning("Invalid field " + key);
      }
    }
  }

  public TunerConstants getTunerConstants() {
    return new TunerConstants(
        ConfigVariables.frontLeftOffset,
        ConfigVariables.frontRightOffset,
        ConfigVariables.backLeftOffset,
        ConfigVariables.backRightOffset);
  }

  public static DynamicRobotConfig loadDynamicRobotConfig() {
    return new DynamicRobotConfig();
  }

  private static void raiseWarning(String warning) {
    logger.warning(warning);
  }
}

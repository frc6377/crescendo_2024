package frc.robot.config;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Robot;
import java.lang.reflect.Field;

/**
 * The DynamicRobotConfig class keeps track of periodically changing constants on the robot
 *
 * <p>This should only have offsets that would change with a mechanical change. There should be no
 * IDs, Conversions, or setpoint information in this config.
 */
public class DynamicRobotConfig {
  private static class ConfigVariables {
    public static double frontLeftOffset = -0.0361328125;
    public static double frontRightOffset = -0.10595703125;
    public static double backLeftOffset = -0.298095703125;
    public static double backRightOffset = 0.2529296875;
    public static double lowGearTurretZero = 0;
    public static double highGearTurretZero = 0;
    public static double limelightXMeters = Units.inchesToMeters(4.34645669);
    public static double limelightYMeters = Units.inchesToMeters(0);
    public static double limelightZMeters = Units.inchesToMeters(17.28346);
    public static double limelightRollInches = Math.toRadians(0);
    public static double limelightPitchInches = Math.toRadians(-18);
    public static double limelightYawInches = Math.toRadians(180);
  }

  private static TunerConstants tunerConstants;

  public DynamicRobotConfig() {
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
    if (tunerConstants == null) {
      tunerConstants =
          new TunerConstants(
              ConfigVariables.frontLeftOffset,
              ConfigVariables.frontRightOffset,
              ConfigVariables.backLeftOffset,
              ConfigVariables.backRightOffset);
    }
    return tunerConstants;
  }

  public void saveTurretZero(TurretZeroConfig cfg) {
    ConfigVariables.lowGearTurretZero = cfg.lowGearTurretZero;
    ConfigVariables.highGearTurretZero = cfg.highGearTurretZero;
    Preferences.setDouble("lowGearTurretZero", cfg.lowGearTurretZero);
    Preferences.setDouble("highGearTurretZero", cfg.lowGearTurretZero);
  }

  public TurretZeroConfig getTurretZeroConfig() {
    return new TurretZeroConfig(
        ConfigVariables.lowGearTurretZero, ConfigVariables.highGearTurretZero);
  }

  public static Transform3d getLimelightTransform() {
    return new Transform3d(new Translation3d(ConfigVariables.limelightXMeters, ConfigVariables.limelightYMeters, ConfigVariables.limelightZMeters), new Rotation3d(ConfigVariables.limelightRollInches, ConfigVariables.limelightPitchInches, ConfigVariables.limelightYawInches));
  }

  private static void raiseWarning(String warning) {
    DriverStation.reportWarning(warning, false);
  }
}

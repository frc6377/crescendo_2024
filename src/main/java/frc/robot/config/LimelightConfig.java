package frc.robot.config;

import edu.wpi.first.math.util.Units;

public class LimelightConfig {
  // These 3 don't make very much sense when our bot has a turret. Also they're wrong. I'm keeping
  // them around so we don't have to refactor Limelight Configs
  public static final double limelightXMeters = Units.inchesToMeters(4.34645669);
  public static final double limelightZMeters = Units.inchesToMeters(17.28346);
  public static final double limelightYawRadians = Math.toRadians(0);

  public static final double limelightYMeters = Units.inchesToMeters(9.99);
  public static final double limelightRollRadians = Math.toRadians(180);
  public static final double limelightPitchRadians = Math.toRadians(90 - 69);
}

package frc.robot.config;

import edu.wpi.first.math.geometry.Pose3d;

public class LimelightConfig {
  public static final Pose3d AMP_CAMERA_DEFAULT_POSITION = null;
  public static final Pose3d TURRET_CAMERA_DEFAULT_POSITION = null;
  public final double limelightXMeters;
  public final double limelightYMeters;
  public final double limelightZMeters;
  public final double limelightRollRadians;
  public final double limelightPitchRadians;
  public final double limelightYawRadians;

  public LimelightConfig(
      double limelightXMeters,
      double limelightYMeters,
      double limelightZMeters,
      double limelightRollRadians,
      double limelightPitchRadians,
      double limelightYawRadians) {
    this.limelightXMeters = limelightXMeters;
    this.limelightYMeters = limelightYMeters;
    this.limelightZMeters = limelightZMeters;
    this.limelightRollRadians = limelightRollRadians;
    this.limelightPitchRadians = limelightPitchRadians;
    this.limelightYawRadians = limelightYawRadians;
  }
}

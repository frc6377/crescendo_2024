package frc.robot.subsystems.vision;

public interface VisionSubsystem {
  public default double getTurretYaw(int id) {
    return 0;
  }

  /**
   * Returns the distance to the april tag in degrees
   *
   * @param id the april to get the distance from
   * @return the distance from the april tag in meters
   */
  public default double getDistance(int id) {
    return 0;
  }
}

package frc.robot.subsystems.vision;

public interface VisionSubsystem {
  public default double getTurretYaw(int id) {
    return 0;
  }

  public default double getDistance(int id) {
    return 0;
  }
}

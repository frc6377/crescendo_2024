package frc.robot.subsystems.vision;

public interface VisionSubsystem {
  public default double getTagYaw(int id, boolean isMain) {
    return 0;
  }

  public default double getTagPitch(int id, boolean isMain) {
    return 0;
  }
}

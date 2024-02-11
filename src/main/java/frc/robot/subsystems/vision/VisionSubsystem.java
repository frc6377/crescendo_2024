package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionSubsystem {
  public Pose3d getPose3d();

  public Pose2d getPose2d();

  public double getTime();
}

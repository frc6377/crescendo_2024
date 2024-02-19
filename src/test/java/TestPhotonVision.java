import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.PhotonSubsystem;
import org.junit.jupiter.api.Test;

public class TestPhotonVision {
  private Pose3d[] currentPoses = {
    new Pose3d(0, 0, 0, null),
    new Pose3d(4, 4, 0, null),
    new Pose3d(3, 3, 0, null),
    new Pose3d(8, 8, 0, null),
    new Pose3d(2, 2, 0, null),
    new Pose3d(2.5, 2.5, 0, null)
  };
  private double[] timestamps = {0, 1, 1.1, 1.15, 2, 2.05};
  private PhotonSubsystem photonSubsystem = new PhotonSubsystem(null);

  @Test
  public void TestPhotonPose() {
    assertTrue(
        photonSubsystem.checkPoseValidity(
            currentPoses[0], currentPoses[1], timestamps[0], timestamps[1]));
    assertFalse(
        photonSubsystem.checkPoseValidity(
            currentPoses[2], currentPoses[3], timestamps[2], timestamps[3]));
    assertTrue(
        photonSubsystem.checkPoseValidity(
            currentPoses[4], currentPoses[5], timestamps[4], timestamps[5]));
  }
}

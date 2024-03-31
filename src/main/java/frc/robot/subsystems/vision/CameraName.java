package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;
import frc.robot.stateManagement.RobotStateManager;

public enum CameraName {
  TURRET(0),
  AMP(1);
  int id;

  private CameraName(int id) {
    this.id = id;
  }

  public Pose3d robotToCamera(RobotStateManager RSM) {
    switch (id) {
      case 0:
        return turretCameraLocationRelToRobot(RSM);
      case 1:
        return ampCameraLocation();
      default:
        // duplicate error reporting because the vision system is going to handle the exception
        // non-fatally
        DriverStation.reportError(
            "Attempt to find camera "
                + this.toString()
                + " location.\nThis is a unknown camera throwing error",
            true);
        throw new UnknownCamera(
            "Attempt to find camera " + this.toString() + " location.\nThis is a unknown camera");
    }
  }

  private Pose3d turretCameraLocationRelToRobot(RobotStateManager RSM) {
    // Might need to invert
    Transform3d accountForTurretRotation =
        new Transform3d(
            new Translation3d(), new Rotation3d(0, 0, RSM.getTurretRotation().getRadians()));
    return VisionConstants.TURRET_CAMERA_POSITION_RELATIVE_TO_ROBOT_CENTER.transformBy(
        accountForTurretRotation);
  }

  private Pose3d ampCameraLocation() {
    throw new UnsupportedOperationException("Unimplemented method 'ampCameraLocation'");
  }

  public static class UnknownCamera extends RuntimeException {
    public UnknownCamera(String err) {
      super(err);
    }
  }
}

package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.PhotonSubsystem.CameraTrackedTarget;

public class VisionMath{
    public Optional<Pose2d> calculateRobotPose(RobotStateManager RSM, CameraTrackedTarget[] targets){
        RSM.getTurretRotation();
        return null;
    }
}
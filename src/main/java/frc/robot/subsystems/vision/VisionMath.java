package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.PhotonSubsystem.CameraName;
import frc.robot.subsystems.vision.PhotonSubsystem.CameraTrackedTarget;

public class VisionMath{
    public Optional<Pose2d> calculateRobotPoseSingleCamera(RobotStateManager RSM, List<CameraTrackedTarget> targets){
        if(targets.size() == 0) return Optional.empty();
        CameraName camera = targets.get(0).camera();
        if(targets.stream().anyMatch((target) -> target.camera() != camera)){
            DriverStation.reportWarning("Invalid Parameters given, Targets only from one camera!", true);
            return Optional.empty();
        }

        if(targets.size() == 1){
            return singleTagPosition(targets.get(0));
        }

        

        return null;
    }

    public Optional<Pose2d> singleTagPosition(CameraTrackedTarget target){
        throw new UnsupportedOperationException("Working on it, gimme a minute");
    }

    private List<Measure<Distance>> tagsToDistance(List<CameraTrackedTarget> targets){
        return targets.stream().map(this::tagToDistance).toList();
    }

    private Measure<Distance> tagToDistance(CameraTrackedTarget target){
        throw new UnsupportedOperationException("Working on it");
    }
}
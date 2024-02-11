package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utilities.DebugEntry;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonSubsystem extends SubsystemBase implements VisionSubsystem {
  private int measurementsUsed = 0;
  private DebugEntry<Integer> measurementEntry = new DebugEntry<Integer>(0, "measurements", this);

  private final BiConsumer<Pose2d, Double> measurementConsumer;

  private PhotonCamera camera;
  private PhotonPipelineResult result;
  private List<PhotonTrackedTarget> targets;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator poseEstimator;

  private DebugEntry<Double> distanceEntryTag3 =
      new DebugEntry<Double>(0.0, "Distance To Tag 3 (m)", this);
  private DebugEntry<Double> distanceEntryTag4 =
      new DebugEntry<Double>(0.0, "Distance To Tag 4 (m)", this);

  public PhotonSubsystem(BiConsumer<Pose2d, Double> measurementConsumer) {
    this.measurementConsumer = measurementConsumer;
    camera = new PhotonCamera("Camera_Module_v1");
    result = camera.getLatestResult();
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    poseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            new Transform3d( //camera offsets will go here when we have them finalized
                new Translation3d(0, 0, 0),
                new Rotation3d(
                    0,
                    Constants.TurretConstants.LIMELIGHT_PITCH_RADIANS,
                    Units.degreesToRadians(180))));
  }

  private Optional<EstimatedRobotPose> getPVEstimatedPose() {
    return poseEstimator.update();
  }

  public Pose3d getPose3d() {
    return getPVEstimatedPose().get().estimatedPose;
  }

  public Pose2d getPose2d() {
    Pose3d estimatedPose = getPVEstimatedPose().get().estimatedPose;
    return new Pose2d(
        estimatedPose.getX(),
        estimatedPose.getY(),
        new Rotation2d(estimatedPose.getRotation().getZ()));
  }

  public double getTime() {
    return getPVEstimatedPose().get().timestampSeconds;
  }

  public void periodic() {
    if (Robot.isReal()) {
      result = camera.getLatestResult();
      if (result.hasTargets()) {
        targets = result.getTargets();
        if (targets.size() > 1) {
          measurementsUsed++;
          measurementConsumer.accept(getPose2d(), getTime());
          if (measurementsUsed % 100 == 0) {
            measurementEntry.log(measurementsUsed);
          }
        }
        // logging stuff
        for (PhotonTrackedTarget target : targets) {
          if (target.getFiducialId() == 3) {
            Pose3d botpose = this.getPose3d();
            double distanceToTag3 =
                Math.sqrt(
                    Math.pow(16.579342 - botpose.getX(), 2)
                        + Math.pow(4.982718 - botpose.getY(), 2)
                        + Math.pow(1.451102 - botpose.getZ(), 2));
            distanceEntryTag3.log(distanceToTag3);
          } else if (target.getFiducialId() == 4) {
            Pose3d botpose = this.getPose3d();
            double distanceToTag4 =
                Math.sqrt(
                    Math.pow(16.579342 - botpose.getX(), 2)
                        + Math.pow(5.547868 - botpose.getY(), 2)
                        + Math.pow(1.451102 - botpose.getZ(), 2));
            distanceEntryTag4.log(distanceToTag4);
          }
        }
      }
    }
  }
}

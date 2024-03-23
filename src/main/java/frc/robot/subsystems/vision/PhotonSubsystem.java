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
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.config.LimelightConfig;
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
  private DebugEntry<Double> measurementEntry = new DebugEntry<Double>(0.0, "measurements", this);

  private final BiConsumer<Pose2d, Double> measurementConsumer;
  private DynamicRobotConfig dynamicRobotConfig;
  private LimelightConfig limelightConfig;
  private Transform3d robotToCam;

  private PhotonCamera mainCamera;
  private PhotonCamera turretCamera;
  private PhotonPipelineResult mainResult;
  private PhotonPipelineResult turretResult;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator poseEstimator;
  private EstimatedRobotPose lastPose;

  private DebugEntry<Double> distanceEntryTag3 =
      new DebugEntry<Double>(0.0, "Distance To Tag 3 (m)", this);
  private DebugEntry<Double> distanceEntryTag4 =
      new DebugEntry<Double>(0.0, "Distance To Tag 4 (m)", this);

  public PhotonSubsystem(BiConsumer<Pose2d, Double> measurementConsumer) {
    this.measurementConsumer = measurementConsumer;
    this.dynamicRobotConfig = new DynamicRobotConfig();
    limelightConfig = dynamicRobotConfig.getLimelightConfig();
    lastPose = new EstimatedRobotPose(new Pose3d(), 0, null, null);
    robotToCam =
        new Transform3d(
            new Translation3d(
                limelightConfig.limelightXMeters,
                limelightConfig.limelightYMeters,
                limelightConfig.limelightZMeters),
            new Rotation3d(
                limelightConfig.limelightRollRadians,
                limelightConfig.limelightPitchRadians,
                limelightConfig.limelightYawRadians));
    mainCamera = new PhotonCamera(Constants.VisionConstants.MAIN_CAMERA_NAME);
    turretCamera = new PhotonCamera(Constants.VisionConstants.TURRET_CAMERA_NAME);
    mainResult = mainCamera.getLatestResult();
    turretResult = turretCamera.getLatestResult();
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    poseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mainCamera, robotToCam);
  }

  private EstimatedRobotPose getPVEstimatedPose() {
    final Optional<EstimatedRobotPose> x = poseEstimator.update();
    if (x.isEmpty()) {
      return lastPose;
    } else {
      lastPose = x.get();
      return lastPose;
    }
  }

  private Pose3d getPose3d() {
    if (lastPose != null) {
      return lastPose.estimatedPose;
    } else {
      return new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    }
  }

  private Pose2d getPose2d() {
    Pose3d estimatedPose = lastPose.estimatedPose;
    return new Pose2d(
        estimatedPose.getX(),
        estimatedPose.getY(),
        new Rotation2d(estimatedPose.getRotation().getZ()));
  }

  private double getTime() {
    return lastPose.timestampSeconds;
  }

  private boolean checkPoseValidity(EstimatedRobotPose pose) {
    double distanceBetweenPoses =
        Math.sqrt(
            Math.pow(pose.estimatedPose.getX() - lastPose.estimatedPose.getX(), 2)
                + Math.pow(pose.estimatedPose.getY() - lastPose.estimatedPose.getY(), 2)
                + Math.pow(
                    pose.estimatedPose.getZ() - lastPose.estimatedPose.getZ(),
                    2)); // 3D distance formula (same one as used in periodic)
    if ((distanceBetweenPoses > Constants.VisionConstants.MAX_ACCEPTABLE_ERROR_METERS)
        && (Math.abs(pose.timestampSeconds - lastPose.timestampSeconds)
            < Constants.VisionConstants.MAX_TIME_BETWEEN_POSES_SECONDS)) {
      System.out.println("POSE REJECTED");
      return false; // pose is invalid
    } else {
      return true; // pose is valid
    }
  }

  public double getTurretYaw(int id) {
    turretResult = turretCamera.getLatestResult();
    if (turretResult.hasTargets()) {
      List<PhotonTrackedTarget> targets = turretResult.getTargets();
      for (PhotonTrackedTarget target : targets) {
        if (target.getFiducialId() == id) {
          return target.getYaw();
        }
      }
    }
    return Double.NaN;
  }

  public double getDistance(int id) {
    turretResult = turretCamera.getLatestResult();
    if (turretResult.hasTargets()) {
      List<PhotonTrackedTarget> targets = turretResult.getTargets();
      for (PhotonTrackedTarget target : targets) {
        if (target.getFiducialId() == id) {
          double ang =
              target.getPitch() + Units.radiansToDegrees(limelightConfig.limelightPitchRadians);
          System.out.println("ang:" + ang);
          return angleToDistanceSpeakerTag(Rotation2d.fromDegrees(ang));
        }
      }
    }
    return 0;
  }

  private double angleToDistanceSpeakerTag(Rotation2d theta) {
    return FieldConstants.SPEAKER_TAG_HEIGHT_METERS / theta.getTan();
  }

  public void periodic() {
    if (Robot.isReal()) {
      mainResult = mainCamera.getLatestResult();
      if (mainResult.hasTargets()) {
        List<PhotonTrackedTarget> targets = mainResult.getTargets();
        if (targets.size() > 1) {
          EstimatedRobotPose newPose = getPVEstimatedPose();
          if ((newPose.estimatedPose.getX() > 12) || (newPose.estimatedPose.getX() < 4.54)) {
            if (checkPoseValidity(newPose)) {
              lastPose = newPose;
            }
            measurementsUsed++;
            measurementConsumer.accept(getPose2d(), getTime());
            if (measurementsUsed % 100 == 0) {
              measurementEntry.log((double) measurementsUsed);
            }
          }
        }
        // logging stuff
        if (!Robot.isCompetition) {
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
}

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.config.LimelightConfig;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.HowdyMath;
import frc.robot.utilities.HowdyMath.Equation3d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
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

  private Map<CameraName, Pose3d> cameraLocations;
  // Provides the camera location relative to the drivetrain
  private ArrayList<Supplier<Pair<Pose3d, CameraName>>> cameraLocationSuppliers;
  private Rotation2d robotRotation;
  private Pose2d previousRobotPosition;

  private DebugEntry<Double> distanceEntryTag3 =
      new DebugEntry<Double>(0.0, "Distance To Tag 3 (m)", this);
  private DebugEntry<Double> distanceEntryTag4 =
      new DebugEntry<Double>(0.0, "Distance To Tag 4 (m)", this);

  public PhotonSubsystem(BiConsumer<Pose2d, Double> measurementConsumer) {
    cameraLocations = new HashMap<>(CameraName.values().length);

    for (CameraName name : CameraName.values()) {
      cameraLocations.put(name, name.getDefaultLocation());
    }

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
    if (turretResult.hasTargets()) {
      List<PhotonTrackedTarget> targets = turretResult.getTargets();
      for (PhotonTrackedTarget target : targets) {
        if (target.getFiducialId() == id) {
          double ang =
              target.getPitch() + Units.radiansToDegrees(limelightConfig.limelightPitchRadians);
          return FieldConstants.SPEAKER_TAG_HEIGHT_METERS / Math.tan(Math.toRadians(ang));
        }
      }
    }
    return 0;
  }

  private double angleToDistanceSpeakerTag(Rotation2d theta) {
    return FieldConstants.SPEAKER_TAG_HEIGHT_METERS / theta.getTan();
  }

  public void addCameraPositionSupplier(Supplier<Pair<Pose3d, CameraName>> supplier) {
    cameraLocationSuppliers.add(supplier);
  }

  public void periodic() {
    if (!Robot.isReal()) return;

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
    turretResult = turretCamera.getLatestResult();

    // Collect all data into one list
    LinkedList<CameraTrackedTarget> allTargets = new LinkedList<>();
    turretResult.targets.forEach(
        (target) -> allTargets.add(new CameraTrackedTarget(target, CameraName.TURRET)));
    mainResult.targets.forEach(
        (target) -> allTargets.add(new CameraTrackedTarget(target, CameraName.AMP_CAMERA)));

    List<Pair<Integer, Measure<Distance>>> distances = calculateDistances(allTargets);
    List<LineOnField> lines = calculateLines(allTargets);

    Translation2d distanceBasedEstimate =
        distancesBasedEstimate(distances, aprilTagFieldLayout, previousRobotPosition);
  }

  public static Translation2d distancesBasedEstimate(
      List<Pair<Integer, Measure<Distance>>> distances,
      AprilTagFieldLayout layout,
      Pose2d initialPose) {
    CircleBasedError errorFunction = new CircleBasedError();

    for (Pair<Integer, Measure<Distance>> distance : distances) {
      Translation2d tagPosition =
          layout.getTagPose(distance.getFirst()).get().toPose2d().getTranslation();
      errorFunction.circles.add(
          Pair.of(
              distance.getSecond().magnitude(), Pair.of(tagPosition.getX(), tagPosition.getY())));
    }

    Pair<Double, Double> robotPosition =
        HowdyMath.gradientDescentOfKnownEquation(
            errorFunction,
            HowdyMath.poseToPair(initialPose),
            VisionConstants.GRADIENT_DESCENT_ITERATIONS);
    return HowdyMath.translation2dFromPair(robotPosition);
  }

  private List<Pair<Integer, Measure<Distance>>> calculateDistances(
      List<CameraTrackedTarget> targets) {
    LinkedList<Pair<Integer, Measure<Distance>>> distances = new LinkedList<>();
    targets.forEach(
        (target) -> {
          try {
            int tagId = target.target.getFiducialId();
            Pose3d tagLocation = aprilTagFieldLayout.getTagPose(tagId).orElseThrow();
            Pose3d cameraLocation = cameraLocations.get(target.name());
            double tagHeightMeters = tagLocation.getZ();
            double cameraHeightMeters = cameraLocation.getZ();
            double deltaHeight = tagHeightMeters - cameraHeightMeters;

            distances.add(
                new Pair<>(tagId, Meters.of(deltaHeight / Math.tan(target.target.getPitch()))));
          } catch (NoSuchElementException e) {
            DriverStation.reportWarning(
                "Can not find tag ID:" + target.target.getFiducialId(), false);
          }
        });
    return distances;
  }

  private List<LineOnField> calculateLines(List<CameraTrackedTarget> targets) {
    LinkedList<LineOnField> lines = new LinkedList<>();
    targets.forEach(
        (target) -> {
          try {
            int tagId = target.target.getFiducialId();
            Pose3d tagLocation = aprilTagFieldLayout.getTagPose(tagId).orElseThrow();
            Pose3d cameraLocation = cameraLocations.get(target.name());
            Rotation2d cameraToTag = Rotation2d.fromRadians(target.target.getYaw());
            Rotation2d robotToTag = cameraToTag.plus(cameraLocation.getRotation().toRotation2d());
            Rotation2d fieldToTag = robotToTag.plus(robotRotation);
            lines.add(new LineOnField(fieldToTag, tagLocation.getTranslation().toTranslation2d()));
          } catch (NoSuchElementException e) {
            DriverStation.reportWarning(
                "Can not find tag ID:" + target.target.getFiducialId(), false);
          }
        });

    return lines;
  }

  private static class CircleBasedError implements Equation3d {
    LinkedList<Pair<Double, Pair<Double, Double>>> circles = new LinkedList<>();

    @Override
    public double calculateZ(Pair<Double, Double> point) {
      double e = 0;
      for (Pair<Double, Pair<Double, Double>> c : circles) {
        e += calculateError(c, point);
      }
      return e;
    }

    @Override
    public Pair<Double, Double> calculateZDerivative(Pair<Double, Double> point) {
      Pair<Double, Double> e = Pair.of(0d, 0d);
      for (Pair<Double, Pair<Double, Double>> c : circles) {
        e = HowdyMath.addPairs(calculateErrorDerivative(c, point), e);
      }
      return e;
    }

    @Override
    public Pair<Double, Double> calculateZSecondDerivative(Pair<Double, Double> point) {
      Pair<Double, Double> e = Pair.of(0d, 0d);
      for (Pair<Double, Pair<Double, Double>> c : circles) {
        e = HowdyMath.addPairs(calculateErrorSecondDerivative(c, point), e);
      }
      return e;
    }

    private double calculateError(
        Pair<Double, Pair<Double, Double>> c, Pair<Double, Double> point) {
      Pair<Double, Double> pointRelToCircle = HowdyMath.subtractPairs(c.getSecond(), point);
      double err =
          (pointRelToCircle.getFirst() * pointRelToCircle.getFirst()
              + pointRelToCircle.getSecond() * pointRelToCircle.getSecond()
              - c.getFirst() * c.getFirst());
      return err * err;
    }

    private Pair<Double, Double> calculateErrorDerivative(
        Pair<Double, Pair<Double, Double>> c, Pair<Double, Double> point) {
      double xDerivative =
          2 * calculateError(c, point) * 2 * (point.getFirst() - c.getSecond().getFirst());
      double yDerivative =
          2 * calculateError(c, point) * 2 * (point.getSecond() - c.getSecond().getSecond());
      return Pair.of(xDerivative, yDerivative);
    }

    private Pair<Double, Double> calculateErrorSecondDerivative(
        Pair<Double, Pair<Double, Double>> c, Pair<Double, Double> point) {
      Pair<Double, Double> firstDerivative = calculateErrorDerivative(c, point);
      double xDerivative =
          4
              * (firstDerivative.getFirst() * (point.getFirst() - c.getSecond().getFirst())
                  + calculateError(c, point));
      double yDerivative =
          4
              * (firstDerivative.getFirst() * (point.getFirst() - c.getSecond().getFirst())
                  + calculateError(c, point));
      return Pair.of(xDerivative, yDerivative);
    }
  }

  private class LineOnField {
    private double dx;
    private double dy;
    private Pair<Double, Double> offSet;

    /**
     * Generates a line off of a known point, and a angle from +y
     *
     * @param direction the angle of the line
     * @param offSet a known intercept of the line in meters
     */
    public LineOnField(Rotation2d direction, Translation2d offSet) {
      this(direction, new Pair<>(offSet.getX(), offSet.getY()));
    }

    /**
     * Generates a line off of a known point, and a angle from +y
     *
     * @param direction
     * @param offSet
     */
    public LineOnField(Rotation2d direction, Pair<Double, Double> offSet) {
      this(direction.getCos(), direction.getSin(), offSet);
    }

    public LineOnField(double dx, double dy, Pair<Double, Double> offSet) {
      this.dx = dx;
      this.dy = dy;
      this.offSet = offSet;
    }
  }

  private record CameraTrackedTarget(PhotonTrackedTarget target, CameraName name) {}

  public static enum CameraName {
    TURRET(0),
    AMP_CAMERA(1);

    private int id;

    private CameraName(int id) {
      this.id = id;
    }

    public Pose3d getDefaultLocation() {
      switch (this.id) {
        case 0:
          return LimelightConfig.TURRET_CAMERA_DEFAULT_POSITION;
        case 1:
          return LimelightConfig.AMP_CAMERA_DEFAULT_POSITION;
      }
      return new Pose3d();
    }
  }
}

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.PhotonSubsystem.CameraTrackedTarget;
import frc.robot.utilities.HowdyMath;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionMath {
  public static class VisionMathConfig {
    public SingleTagMethod singleTagMethod = SingleTagMethod.NONE;
    public TwoTagMethod twoTagMethod = TwoTagMethod.INTERCEPTING_CIRCLE;
    public NTagMethod nTagMethod = NTagMethod.NONE;
  }

  /**
   * Calculates the robot position with default vision math configuration. To be distributed to the
   * correct position calcuation method based on the default vision config. Will return an empty
   * optional if no method is applicable.
   *
   * @param RSM the robot state
   * @param targets the targets in view
   * @param layout the lay out of april tags
   * @return the approximate location of the robot.
   */
  public static Optional<Pose2d> calculateRobotPoseSingleCamera(
      RobotStateManager RSM, List<CameraTrackedTarget> targets, AprilTagFieldLayout layout) {
    return calculateRobotPoseSingleCamera(RSM, targets, layout, new VisionMathConfig());
  }

  /**
   * Calculates the robot position with the given vision math configuration. To be distributed to
   * the correct position calcuation method based on the default vision config. Will return an empty
   * optional if no method is applicable.
   *
   * @param RSM the robot state
   * @param targets the targets in view
   * @param layout the lay out of april tags
   * @param visionConfig the vision configuration to use
   * @return the approximate location of the robot. if a target is in view
   */
  public static Optional<Pose2d> calculateRobotPoseSingleCamera(
      RobotStateManager RSM,
      List<CameraTrackedTarget> targets,
      AprilTagFieldLayout layout,
      VisionMathConfig visionConfig) {
    if (targets.size() == 0) return Optional.empty();
    // This is a safe access due to the above check
    CameraName camera = targets.get(0).camera();
    if (targets.stream().anyMatch((target) -> target.camera() != camera)) {
      DriverStation.reportWarning("Invalid Parameters given, Targets only from one camera!", true);
      return Optional.empty();
    }

    // Handle Single Tag Check
    if (targets.size() == 1) {
      switch (visionConfig.singleTagMethod) {
        case DISTANCE_LINE:
          return lineDistanceMethodSingleTarget(targets.get(0));
        default:
          DriverStation.reportWarning("Unknown One Tag Method Selected!", true);
        case NONE:
          break;
      }
    }

    if (targets.size() == 2) {
      switch (visionConfig.twoTagMethod) {
        case INTERCEPTING_CIRCLE:
          return cirlceBasedTwoTagPosition(targets, layout, RSM);
        default:
          DriverStation.reportWarning("Unknown Two Tag Method Selected!", true);
        case NONE: // Fall through is intentional
          break;
      }
    }

    switch (visionConfig.nTagMethod) {
      case NONE:
        return Optional.empty();
      default:
        DriverStation.reportWarning("Unknown NTag Method Selected!", true);
        return Optional.empty();
    }
  }

  /**
   * Esitmates the robot location by estimating ythe distance to two tags.
   *
   * <p>Then solving for where the two distances intercept, and discarding the one that is behind
   * the tags.
   *
   * <p>Based off of solution presented here {@link
   * https://www.johndcook.com/blog/2023/08/27/intersect-circles/}
   *
   * @param targets the targets to calculate robot position off of. Must comprise only 2 tags from
   *     the same camera
   * @param layout the layout representing where all the tags are
   * @param RSM the current robot state
   * @return the estiamted robot position with no filtering for possiblity
   */
  private static Optional<Pose2d> cirlceBasedTwoTagPosition(
      List<CameraTrackedTarget> targets, AprilTagFieldLayout layout, RobotStateManager RSM) {
    if (targets.size() != 2) {
      DriverStation.reportWarning(
          String.format(
              "Warning - Invalid calling two tag position with incorrect number of targets (%d targets)",
              targets.size()),
          true);
      return Optional.empty();
    }

    CameraName cameraInUse = targets.get(0).camera();
    if (targets.stream().anyMatch((a) -> a.camera() != cameraInUse)) {
      DriverStation.reportWarning("Invalid Parameters given, Targets only from one camera!", true);
      return Optional.empty();
    }

    Pose3d cameraPositionRelToRobot = cameraInUse.robotToCamera(RSM);

    PhotonTrackedTarget trackedA = targets.get(0).target();
    PhotonTrackedTarget trackedB = targets.get(1).target();

    Distance distanceToTagAUnitRep = tagToCameraDistance(targets.get(0), layout, RSM);
    Distance distanceToTagBUnitRep = tagToCameraDistance(targets.get(1), layout, RSM);

    double distanceToTagA = distanceToTagAUnitRep.in(Meters);
    double distanceToTagB = distanceToTagBUnitRep.in(Meters);

    SmartDashboard.putNumber("distance A ", distanceToTagA);
    SmartDashboard.putNumber("distance B ", distanceToTagB);

    Translation2d tagLocationA =
        layout
            .getTagPose(trackedA.getFiducialId())
            .orElseThrow()
            .getTranslation()
            .toTranslation2d();
    Translation2d tagLocationB =
        layout
            .getTagPose(trackedB.getFiducialId())
            .orElseThrow()
            .getTranslation()
            .toTranslation2d();

    double distanceTagAToB = tagLocationA.getDistance(tagLocationB);

    // Circles don't intercept
    if (distanceTagAToB > distanceToTagA + distanceToTagB) {
      return Optional.empty();
    }

    // The distance from circle A towards circle B to travel.
    double aTowardsBDistance =
        (distanceTagAToB * distanceTagAToB
                - distanceToTagB * distanceToTagB
                + distanceToTagA * distanceToTagA)
            / (2 * distanceTagAToB);

    // y squared.
    // the distance normal to A towards B
    double numberatorPartA =
        4 * distanceTagAToB * distanceTagAToB * distanceToTagA * distanceToTagA;
    double numeratorPartB =
        (distanceTagAToB * distanceTagAToB
            - distanceToTagB * distanceToTagB
            + distanceToTagA * distanceToTagA);
    double y2 =
        (numberatorPartA - numeratorPartB * numeratorPartB)
            / (4 * distanceTagAToB * distanceTagAToB);
    double ya = Math.sqrt(y2);
    double yb = -ya;

    // TODO: Check both possible locations for feasiblity besides deafault
    Translation2d pointRelativeTagAToBOptA = new Translation2d(aTowardsBDistance, ya);
    Translation2d pointRelativeTagAToBOptB = new Translation2d(aTowardsBDistance, yb);

    Rotation2d circleSpaceToRealRotation = tagLocationB.minus(tagLocationA).getAngle();
    Translation2d circleSpaceToRealTranslation = tagLocationA;

    // Currently arbitraily using point A
    Translation2d robotTranslation =
        pointRelativeTagAToBOptA
            .rotateBy(circleSpaceToRealRotation)
            .plus(circleSpaceToRealTranslation);

    Pose2d cameraPosition =
        new Pose2d(robotTranslation, cameraPositionRelToRobot.getRotation().toRotation2d());
    Transform2d cameraToRobot =
        HowdyMath.pose2dToTransform2d(cameraPositionRelToRobot.toPose2d()).inverse();
    Pose2d robotPosition = cameraPosition.transformBy(cameraToRobot);

    // This technique doesn't generate a robot rotation, So just repeating the already known one.
    return Optional.of(new Pose2d(robotPosition.getTranslation(), RSM.getRobotRotation()));
  }

  /**
   * Approximates the robtot location using a single tag.
   *
   * @param target
   * @return
   */
  public static Optional<Pose2d> lineDistanceMethodSingleTarget(CameraTrackedTarget target) {
    // To use PhotonUtils.estimateFieldToRobot
    return Optional.empty();
  }

  /**
   * Uses a list of targets and calculates the cameras distance to each. the distance is the
   * distance to the camera that sees it.
   *
   * @param targets the targets to calculate the distance to
   * @param layout the layout of apriltags
   * @param RSM the current robot state
   * @return the distance to each tag, in the same order given
   */
  public static List<Distance> tagsToCameraDistance(
      List<CameraTrackedTarget> targets, AprilTagFieldLayout layout, RobotStateManager RSM) {
    return targets.stream().map((a) -> VisionMath.tagToCameraDistance(a, layout, RSM)).toList();
  }

  /**
   * Calculates the distance from a CameraTrackedObject to the camera
   *
   * @param cameraTarget the target
   * @param layout the layout of april tags
   * @param RSM the current robot state
   * @return the distance from the april tag
   */
  private static Distance tagToCameraDistance(
      CameraTrackedTarget cameraTarget, AprilTagFieldLayout layout, RobotStateManager RSM) {
    PhotonTrackedTarget target = cameraTarget.target();
    Pose3d tagLocation = layout.getTagPose(target.getFiducialId()).orElseThrow();
    Pose3d cameraLocation = cameraTarget.camera().robotToCamera(RSM);

    SmartDashboard.putNumber(
        "ID " + target.getFiducialId() + "at deg",
        Units.radiansToDegrees(
            cameraLocation.getRotation().getAngle() + Units.degreesToRadians(target.getPitch())));
    return Meters.of(
        PhotonUtils.calculateDistanceToTargetMeters(
            cameraLocation.getZ(),
            tagLocation.getZ(),
            cameraLocation.getRotation().getAngle(),
            Units.degreesToRadians(target.getPitch())));
  }

  public enum SingleTagMethod {
    NONE,
    DISTANCE_LINE,
  }

  public enum TwoTagMethod {
    NONE,
    INTERCEPTING_CIRCLE,
  }

  public enum NTagMethod {
    NONE,
  }
}

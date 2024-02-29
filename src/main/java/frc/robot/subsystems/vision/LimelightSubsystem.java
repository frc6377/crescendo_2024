// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.LimelightHelpers;
import java.util.function.BiConsumer;

public class LimelightSubsystem extends VisionSubsystem {
  private LimelightHelpers.LimelightResults results;

  private int measurementsUsed = 0;
  private DebugEntry<Integer> measurementEntry = new DebugEntry<Integer>(0, "measurements", this);
  private DebugEntry<Double> distanceEntryTag3 =
      new DebugEntry<Double>(0.0, "Tag 3 Distance (m)", this);
  private DebugEntry<Double> distanceEntryTag4 =
      new DebugEntry<Double>(0.0, "Tag 4 Distance (m)", this);

  private int lastHeartbeat = 0;

  private final RobotStateManager robotStateManager;

  private final BiConsumer<Pose2d, Double> measurementConsumer;

  public LimelightSubsystem(
      BiConsumer<Pose2d, Double> measurementConsumer, RobotStateManager robotStateManager) {
    results = LimelightHelpers.getLatestResults("");
    this.measurementConsumer = measurementConsumer;
    this.robotStateManager = robotStateManager;
    LimelightHelpers.setLEDMode_ForceOff("");
    LimelightHelpers.setStreamMode_PiPMain("");
  }

  private double getTagCount() {
    return results.targetingResults.targets_Fiducials.length;
  }

  private Pose3d getPose3d() {
    Pose3d botpose = LimelightHelpers.getBotPose3d_wpiBlue("");
    double distanceToTag3 =
        Math.sqrt(
            Math.pow(16.579342 - botpose.getX(), 2)
                + Math.pow(4.982718 - botpose.getY(), 2)
                + Math.pow(1.451102 - botpose.getZ(), 2));
    double distanceToTag4 =
        Math.sqrt(
            Math.pow(16.579342 - botpose.getX(), 2)
                + Math.pow(5.547868 - botpose.getY(), 2)
                + Math.pow(1.451102 - botpose.getZ(), 2));
    distanceEntryTag3.log(distanceToTag3);
    distanceEntryTag4.log(distanceToTag4);
    return botpose;
  }

  private Pose2d getPose2d() {
    double[] botpose = LimelightHelpers.getBotPose_wpiBlue("");
    return new Pose2d(botpose[0], botpose[1], new Rotation2d(Units.degreesToRadians(botpose[5])));
  }

  private double getTime() {
    // Accounts for latency
    // (https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization)
    return Timer.getFPGATimestamp()
        - (LimelightHelpers.getLatency_Capture("") / 1000.0)
        - (LimelightHelpers.getLatency_Pipeline("") / 1000.0);
  }

  public double getTurretYaw(int ID) {
    if (ID
        == ((robotStateManager.getAllianceColor() == AllianceColor.BLUE)
            ? Constants.TurretConstants.SPEAKER_TAG_ID_BLUE
            : Constants.TurretConstants.SPEAKER_TAG_ID_RED)) {
      return LimelightHelpers.getTX("limelight");
    }
    return 0;
  }

  public double getTurretPitch(int ID) {
    if (ID
        == ((robotStateManager.getAllianceColor() == AllianceColor.BLUE)
            ? Constants.TurretConstants.SPEAKER_TAG_ID_BLUE
            : Constants.TurretConstants.SPEAKER_TAG_ID_RED)) {
      return LimelightHelpers.getTY("limelight");
    }
    return 0;
  }

  private int getHeartbeat() {
    // "hb" gets the id of the current network table frame
    return (int)
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("hb").getInteger(0);
  }

  public Pose3d getNoteLocation() {
    return null; // Replace with Coral functionality
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      results = LimelightHelpers.getLatestResults("");
      int heartbeat = getHeartbeat();
      if (heartbeat != lastHeartbeat) {
        lastHeartbeat = heartbeat;
        if (getTagCount() > 1) {
          measurementsUsed++;
          measurementConsumer.accept(getPose2d(), getTime());
          if (measurementsUsed % 100 == 0) {
            measurementEntry.log(measurementsUsed);
          }
        }
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

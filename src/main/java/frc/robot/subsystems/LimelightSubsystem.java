// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.networktables.DebugEntry;
import java.util.function.BiConsumer;

public class LimelightSubsystem extends SubsystemBase {
  private LimelightHelpers.LimelightResults results;

  private int measurementsUsed = 0;
  private DebugEntry<Integer> measurementEntry = new DebugEntry<Integer>(0, "measurements", this);
  private DebugEntry<Double> distanceEntryTag3 =
      new DebugEntry<Double>(0.0, "Tag 3 Distance (m)", this);
  private DebugEntry<Double> distanceEntryTag4 =
      new DebugEntry<Double>(0.0, "Tag 4 Distance (m)", this);

  private int lastHeartbeat = 0;

  private final BiConsumer<Pose2d, Double> measurementConsumer;

  public LimelightSubsystem(BiConsumer<Pose2d, Double> measurementConsumer) {
    results = LimelightHelpers.getLatestResults("");

    this.measurementConsumer = measurementConsumer;
    // LimelightHelpers.setLEDMode_ForceOff("");
    LimelightHelpers.setStreamMode_PiPMain("");
  }

  public double getTagCount() {
    return results.targetingResults.targets_Fiducials.length;
  }

  private Pose2d getPose2d() {
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
    return botpose.toPose2d();
    // return new Pose2d(botpose[0], botpose[1], new
    // Rotation2d(Units.degreesToRadians(botpose[5])));
  }

  private double getTime() {
    // Accounts for latency
    // (https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization)
    return Timer.getFPGATimestamp()
        - (LimelightHelpers.getLatency_Capture("") / 1000.0)
        - (LimelightHelpers.getLatency_Pipeline("") / 1000.0);
  }

  private int getHeartbeat() {
    // "hb" gets the id of the current network table frame
    return (int)
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("hb").getInteger(0);
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

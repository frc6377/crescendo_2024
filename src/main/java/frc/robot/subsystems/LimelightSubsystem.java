// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.utilities.DebugEntry;
import java.util.function.BiConsumer;

public class LimelightSubsystem extends SubsystemBase {
  private LimelightHelpers.LimelightResults results;

  private int measurementsUsed = 0;
  private DebugEntry<Integer> measurementEntry = new DebugEntry<Integer>(0, "measurements", this);

  private final BiConsumer<Pose2d, Double> measurementConsumer;

  public LimelightSubsystem(BiConsumer<Pose2d, Double> measurementConsumer) {
    results = LimelightHelpers.getLatestResults("");

    this.measurementConsumer = measurementConsumer;
    LimelightHelpers.setLEDMode_ForceOff("");
    LimelightHelpers.setStreamMode_PiPMain("");
  }

  public double getTagCount() {
    return results.targetingResults.targets_Fiducials.length;
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

  @Override
  public void periodic() {
    results = LimelightHelpers.getLatestResults("");

    if (getTagCount() > 1) {
      measurementsUsed++;
      measurementConsumer.accept(getPose2d(), getTime());
      if (measurementsUsed % 100 == 0) {
        measurementEntry.log(measurementsUsed);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

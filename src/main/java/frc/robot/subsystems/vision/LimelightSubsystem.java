// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  private LimelightHelpers.LimelightResults results;

  private RobotStateManager robotStateManager;

  private DebugEntry<Double> distanceEntryTag3 =
      new DebugEntry<Double>(0.0, "Tag 3 Distance (m)", this);
  private DebugEntry<Double> distanceEntryTag4 =
      new DebugEntry<Double>(0.0, "Tag 4 Distance (m)", this);

  public LimelightSubsystem(RobotStateManager robotStateManager) {
    results = LimelightHelpers.getLatestResults("");
    this.robotStateManager = robotStateManager;
    LimelightHelpers.setLEDMode_ForceOff("");
    LimelightHelpers.setStreamMode_PiPMain("");
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

  public double getDistanceToTag(int ID) {
    if (ID == Constants.TurretConstants.SPEAKER_TAG_ID_BLUE) {
      return getPose2d().getX() - Constants.FieldConstants.BLUE_SPEAKER.getX();
    } else if (ID == Constants.TurretConstants.SPEAKER_TAG_ID_RED) {
      return Constants.FieldConstants.RED_SPEAKER.getX() - getPose2d().getX();
    }
    return 0;
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

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

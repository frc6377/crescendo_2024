// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.TOFSensorSimple;
import java.util.function.BooleanSupplier;

public class SensorSubsystem extends SubsystemBase {
  // Beam Breaks
  private final TOFSensorSimple sourceBreak;
  private final TOFSensorSimple groundBreak;

  private final RobotStateManager robotStateManager;

  private DebugEntry<Boolean> sourceLog;
  private DebugEntry<Boolean> groundLog;
  private DebugEntry<Boolean> baseLog;
  private DebugEntry<Boolean> scoringLog;

  private ShuffleboardTab SensorsTab = Shuffleboard.getTab("Sensors");
  private GenericEntry sourceBreakDis = SensorsTab.add("Source Break Distance", 0).getEntry();
  private GenericEntry groundBreakDis = SensorsTab.add("Ground Break Distance", 0).getEntry();

  /** Creates a new TrapArm. */
  public SensorSubsystem(RobotStateManager robotStateManager) {
    this.robotStateManager = robotStateManager;

    // Initialize Sensors
    sourceBreak =
        new TOFSensorSimple(
            Constants.SensorConstants.SOURCE_BREAK_ID, Constants.SensorConstants.WRIST_BREAK_THOLD);
    groundBreak =
        new TOFSensorSimple(
            Constants.SensorConstants.GROUND_BREAK_ID, Constants.SensorConstants.WRIST_BREAK_THOLD);

    // SmartDashboard
    sourceLog = new DebugEntry<Boolean>(sourceBreak.isBeamBroke(), "Source Beam Break", this);
    sourceBreakDis.setDouble(sourceBreak.getMilliMeters());

    // Simulation
    if (Robot.isSimulation()) {
      // TODO: Do simulation if that's even possible
    }
  }

  // Boolean Suppliers
  public BooleanSupplier getSourceBreakBool() {
    return () -> sourceBreak.isBeamBroke();
  }

  public BooleanSupplier getSourceBreakBoolInverse() {
    return () -> !sourceBreak.isBeamBroke();
  }

  public BooleanSupplier getGroundBreakBool() {
    return () -> groundBreak.isBeamBroke();
  }

  public BooleanSupplier getGroundBreakBoolInverse() {
    return () -> !groundBreak.isBeamBroke();
  }

  public TOFSensorSimple getSourceBreak() {
    return sourceBreak;
  }

  // Commands

  @Override
  public void periodic() {
    sourceLog.log(sourceBreak.isBeamBroke());
    groundLog.log(groundBreak.isBeamBroke());
  }

  @Override
  public void simulationPeriodic() {}
}

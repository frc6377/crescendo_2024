// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.utilities.TOFSensorSimple;
import java.util.function.BooleanSupplier;

public final class SensorManager {
  // Beam Breaks
  private static final TOFSensorSimple sourceBreak =
      new TOFSensorSimple(
          Constants.SensorConstants.SOURCE_BREAK_ID, Constants.SensorConstants.WRIST_BREAK_THOLD);
  ;

  // Boolean Suppliers
  public static final BooleanSupplier getSourceBreakBool() {
    return () -> sourceBreak.isBeamBroken();
  }

  public static final BooleanSupplier getSourceBreakBoolInverse() {
    return () -> !sourceBreak.isBeamBroken();
  }

  public static final Trigger getSourceBreakTrigger() {
    return new Trigger(getSourceBreakBool());
  }
}

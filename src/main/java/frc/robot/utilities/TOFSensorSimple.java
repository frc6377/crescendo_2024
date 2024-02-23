// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

/** Add your docs here. */
public class TOFSensorSimple {
  private TimeOfFlight sensor;
  private double threshold;
  private boolean isBeamBrokeSim;

  public TOFSensorSimple(int ID, double threshold) {
    sensor = new TimeOfFlight(ID);
    this.threshold = threshold; // in mm
    isBeamBrokeSim = false;
  }

  public TOFSensorSimple(int ID) {
    sensor = new TimeOfFlight(ID);
    this.threshold = 1; // in mm
  }

  public void updateSimBool(boolean isBeamBroke) {
    isBeamBrokeSim = isBeamBroke;
  }

  public double getMilliMeters() {
    return sensor.getRange();
  }

  public boolean isBeamBroken() {
    if (Robot.isReal()) {
      return getMilliMeters() < threshold;
    }
    return isBeamBrokeSim;
  }

  public Trigger beamBroken(Command action) {
    return new Trigger(this::isBeamBroken);
  }

  public void blink() {
    sensor.identifySensor();
  }
}

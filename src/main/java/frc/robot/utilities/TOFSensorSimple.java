// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class TOFSensorSimple {
  private TimeOfFlight sensor;
  private double threshold;

  public TOFSensorSimple(int ID, double threshold) {
    sensor = new TimeOfFlight(ID);
    this.threshold = threshold; // in mm
  }

  public double getMillameters() {
    return sensor.getRange();
  }

  public boolean isBeamBroke() {
    return getMillameters() < threshold;
  }

  public Trigger beamBroken(Command action) {
    return new Trigger(this::isBeamBroke);
  }

  public void blink() {
    sensor.identifySensor();
  }
}

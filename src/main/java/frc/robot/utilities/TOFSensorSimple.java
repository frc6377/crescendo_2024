// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class TOFSensorSimple {
  private TimeOfFlight Sensor;
  private double threshold;

  public TOFSensorSimple(int ID, double threshold) {
    Sensor = new TimeOfFlight(ID);
    this.threshold = threshold;
  }

  public double getDistance() {
    return Sensor.getRange();
  }

  public boolean isBeamBroke() {
    return getDistance() < threshold;
  }

  public void identify() {
    Sensor.identifySensor();
  }

  public void putBuilder(SendableBuilder builder) {
    Sensor.initSendable(builder);
  }
}

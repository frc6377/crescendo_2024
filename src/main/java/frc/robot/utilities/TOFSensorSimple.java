// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

/** Add your docs here. */
public class TOFSensorSimple {
  private ShuffleboardTab sensorTab;
  private TimeOfFlight sensor;
  private double threshold;
  private int id;

  public TOFSensorSimple(int ID, double threshold) {
    sensor = new TimeOfFlight(ID);
    // sensor.setRangingMode(RangingMode.Short, 100);
    this.threshold = threshold; // in mm
    this.id = ID;
  }

  public double getMilliMeters() {
    if (!Robot.isCompetition) {}

    return sensor.getRange();
  }

  public boolean get() {
    return isBeamBroke();
  }

  public boolean isBeamBroke() {
    return getMilliMeters() < threshold;
  }

  public Trigger beamBroken(Command action) {
    return new Trigger(this::isBeamBroke);
  }

  public void blink() {
    sensor.identifySensor();
  }
}

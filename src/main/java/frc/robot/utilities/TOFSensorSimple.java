// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

/** Add your docs here. */
public class TOFSensorSimple {
  private static ShuffleboardTab sensorTab = Shuffleboard.getTab("sensors");
  private TimeOfFlight sensor;
  private double threshold;
  private int id;

  public TOFSensorSimple(int ID, double threshold) {
    this.id = ID;
    sensor = new TimeOfFlight(this.id);
    if (!Robot.isCompetition) {
      sensorTab.addDouble("tof sensor " + this.id + " distance (mm)", this::getMilliMeters);
      sensorTab.addBoolean("tof sensor " + this.id + " broken", this::get);
    }
    this.threshold = threshold; // in mm
  }

  public int getID() {
    return this.id;
  }

  public double getMilliMeters() {
    if (!Robot.isCompetition) {}

    return this.sensor.getRange();
  }

  public boolean get() {
    return isBeamBroke();
  }

  public boolean isBeamBroke() {
    return getMilliMeters() < this.threshold;
  }

  public Trigger beamBroken(Command action) {
    return new Trigger(this::isBeamBroke);
  }

  public void blink() {
    sensor.identifySensor();
  }
}

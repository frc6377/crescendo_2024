// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

/** Add your docs here. */
public class TOFSensorSimple {
  private static ShuffleboardTab sensorTab = Shuffleboard.getTab("sensors");
  private DebugEntry<Double> TOFDistance;
  private DebugEntry<Boolean> TOFBroken;
  private TimeOfFlight sensor;
  private double threshold;
  private int id;

  public TOFSensorSimple(int ID, double threshold, Subsystem subsystem) {
    sensor = new TimeOfFlight(ID);
    TOFDistance =
        new DebugEntry<Double>(
            this.getMilliMeters(), "TOF sensor " + ID + " distance (mm)", subsystem);
    TOFBroken = new DebugEntry<Boolean>(this.get(), "TOF sensor " + ID + " broken", subsystem);
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

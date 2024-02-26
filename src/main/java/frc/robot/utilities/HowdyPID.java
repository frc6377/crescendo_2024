// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Add your docs here. */
public class HowdyPID {
  private double P;
  private double I;
  private double D;
  private double Iz; // I Zone
  private double FF;

  public HowdyPID(double p, double i, double d, double iz) {
    this.P = p;
    this.I = i;
    this.D = d;
    this.Iz = iz;
    this.FF = 0.0;
  }

  public HowdyPID(double p, double i, double d, double iz, double ff) {
    this.P = p;
    this.I = i;
    this.D = d;
    this.Iz = iz;
    this.FF = ff;
  }

  public double getP() {
    return this.P;
  }

  public double getI() {
    return this.I;
  }

  public double getD() {
    return this.D;
  }

  public double getIz() {
    return this.Iz;
  }

  public double getFF() {
    return this.FF;
  }
}

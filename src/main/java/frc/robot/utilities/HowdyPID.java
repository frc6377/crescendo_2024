// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class HowdyPID {
  private double P;
  private double I;
  private double D;
  private double Iz; // I Zone
  private double FF;

  private TunableNumber tuneP;
  private TunableNumber tuneI;
  private TunableNumber tuneD;
  private TunableNumber tuneIz;

  public HowdyPID(double p, double i, double d) {
    this.P = p;
    this.I = i;
    this.D = d;
    this.Iz = 0.0;
    this.FF = 0.0;
  }

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

  public void createTunableNumbers(String name, CANSparkMax motor) {
    this.tuneP = new TunableNumber(name.concat(" P"), this.P, p -> motor.getPIDController().setP(p));
    this.tuneI = new TunableNumber(name.concat(" I"), this.P, i -> motor.getPIDController().setI(i));
    this.tuneD = new TunableNumber(name.concat(" D"), this.P, d -> motor.getPIDController().setD(d));
    this.tuneIz = new TunableNumber(name.concat(" Iz"), this.P, iz -> motor.getPIDController().setIZone(iz));
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

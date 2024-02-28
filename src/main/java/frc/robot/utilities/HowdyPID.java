// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Add your docs here. */
public class HowdyPID{
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

  public void createTunableNumbers(String name, CANSparkMax motor, Subsystem subsystem) {
    this.tuneP =
        new TunableNumber(
            name.concat(" P"), this.P, p -> motor.getPIDController().setP(p), subsystem);
    this.tuneI =
        new TunableNumber(
            name.concat(" I"), this.P, i -> motor.getPIDController().setI(i), subsystem);
    this.tuneD =
        new TunableNumber(
            name.concat(" D"), this.P, d -> motor.getPIDController().setD(d), subsystem);
    this.tuneIz =
        new TunableNumber(
            name.concat(" Iz"), this.P, iz -> motor.getPIDController().setIZone(iz), subsystem);
  }

  public void createTunableNumbers(String name, PIDController controller, Subsystem subsystem) {
    this.tuneP =
        new TunableNumber(
            name.concat(" P"), this.P, p -> controller.setP(p), subsystem);
    this.tuneI =
        new TunableNumber(
            name.concat(" I"), this.P, i -> controller.setI(i), subsystem);
    this.tuneD =
        new TunableNumber(
            name.concat(" D"), this.P, d -> controller.setD(d), subsystem);
    this.tuneIz =
        new TunableNumber(
            name.concat(" Iz"), this.P, iz -> controller.setIZone(iz), subsystem);
  }

  public SparkPIDController getPidController(CANSparkMax motor) {
    SparkPIDController controller = motor.getPIDController();
    controller.setP(this.P);
    controller.setI(this.I);
    controller.setD(this.D);
    controller.setIZone(this.Iz);
    controller.setFF(this.FF);
    return controller;
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

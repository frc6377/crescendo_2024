// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package howdyutilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

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
  private TunableNumber tuneFF;

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
            name.concat(" I"), this.I, i -> motor.getPIDController().setI(i), subsystem);
    this.tuneD =
        new TunableNumber(
            name.concat(" D"), this.D, d -> motor.getPIDController().setD(d), subsystem);
    this.tuneIz =
        new TunableNumber(
            name.concat(" Iz"), this.Iz, iz -> motor.getPIDController().setIZone(iz), subsystem);
    this.tuneFF =
        new TunableNumber(
            name.concat(" FF"), this.FF, ff -> motor.getPIDController().setFF(ff), subsystem);
  }

  public void createTunableNumbers(String name, PIDController controller, Subsystem subsystem) {
    this.tuneP = new TunableNumber(name.concat(" P"), this.P, p -> controller.setP(p), subsystem);
    this.tuneI = new TunableNumber(name.concat(" I"), this.I, i -> controller.setI(i), subsystem);
    this.tuneD = new TunableNumber(name.concat(" D"), this.D, d -> controller.setD(d), subsystem);
    this.tuneIz =
        new TunableNumber(name.concat(" Iz"), this.Iz, iz -> controller.setIZone(iz), subsystem);
  }

  public void setSparkPidController(CANSparkMax motor) {
    motor.getPIDController().setP(this.P);
    motor.getPIDController().setI(this.I);
    motor.getPIDController().setD(this.D);
    motor.getPIDController().setIZone(this.Iz);
    motor.getPIDController().setFF(this.FF);
  }

  public void setSparkPidController(CANSparkMaxSim motor) {
    motor.getPIDController().setP(this.P);
    motor.getPIDController().setI(this.I);
    motor.getPIDController().setD(this.D);
    motor.getPIDController().setIZone(this.Iz);
    motor.getPIDController().setFF(this.FF);
  }

  public PIDController getPIDController() {
    final PIDController controller = new PIDController(this.P, this.I, this.D);
    controller.setIZone(this.Iz);
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

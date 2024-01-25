// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

/** Add your docs here. */
public class CANSparkMaxSim {

  public static final double kPeriod = 0.001; // 1kHz
  private SparkPIDController ctrl;
  private double prevError, IState, setpoint, position, velocity, output;
  private boolean stopped;
  private ControlType type;

  public CANSparkMaxSim(CANSparkMax spark) {
    ctrl = spark.getPIDController();
    setpoint = 0;
    position = 0;
    velocity = 0;
    this.type = ControlType.kVelocity;
    IState = 0;
    output = 0;
    stopped = true;
  }

  public void stopMotor() {
    stopped = true;
    output = 0;
    IState = 0;
    prevError = 0;
  }

  public void setSetpoint(double setpoint, ControlType t) {
    stopped = false;
    this.type = t;
    this.setpoint = setpoint;
  }

  public void update(double radsPerSec) {
    velocity = radsPerSec / (2 * Math.PI) * 60; // RPM
    position += radsPerSec / (2 * Math.PI) * kPeriod;
    calculate();
  }

  public double getOutput() {
    return output;
  }

  // Algorithm pulled from https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
  private double calculate() {
    if (stopped) {
      return 0;
    }
    double measurement = 0;
    if (type == ControlType.kVelocity) {
      measurement = velocity;
    } else if (type == ControlType.kPosition) {
      measurement = position;
    }
    double error = setpoint - measurement;
    double P_Err = error * ctrl.getP();
    if (Math.abs(error) <= ctrl.getIZone() || ctrl.getIZone() == 0d) {
      IState += error * ctrl.getI();
    } else {
      IState = 0;
    }

    double D_err = error - prevError;
    prevError = error;
    D_err *= ctrl.getD();
    double F_err = setpoint * ctrl.getFF();
    output = P_Err + IState + D_err + F_err;
    output = Math.min(Math.max(output, ctrl.getOutputMin()), ctrl.getOutputMax());
    return output;
  }
}

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
  private double prevError, IState, setpoint, measurement;
  private ControlType type;

  public CANSparkMaxSim(CANSparkMax spark, ControlType type) {
    ctrl = spark.getPIDController();
    setpoint = 0;
    measurement = 0;
    this.type = type;
    IState = 0;
  }

  public void setControlType(ControlType control) {
    type = control;
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void update(double radsPerSec) {
    if (type == ControlType.kVelocity) {
      measurement = radsPerSec / (2 * Math.PI) * 60; // RPM
    } else if (type == ControlType.kPosition) {
      measurement += radsPerSec / (2 * Math.PI) * kPeriod;
    }
  }

  // Algorithm pulled from https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
  public double get() {
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
    double output = P_Err + IState + D_err + F_err;
    output = Math.min(Math.max(output, ctrl.getOutputMin()), ctrl.getOutputMax());
    return output;
  }
}

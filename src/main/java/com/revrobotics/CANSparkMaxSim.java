// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Pretending to be apart of revrobotics package so we can extend
 * classes with protected constructors (SparkPIDController and SparkRelativeEncoder)
 */
package com.revrobotics;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Robot;

/*
 * Methods to add Sim functions to:
 *    CANSparkMax:
 *      getPIDController
 *      getEncoder
 *      get
 *      stopMotor
 *    SparkPIDController:
 *      setReference
 *    RelativeEncoder:
 *      getPosition
 *      getVelocity
 *      setPosition
 */

public class CANSparkMaxSim extends CANSparkMax {

  public class SparkPIDControllerSim extends SparkPIDController implements Sendable {
    private static int instances = 0;
    private double P, I, D, FF, IZone;

    SparkPIDControllerSim(CANSparkBase device) {
      super(device);
      instances++;
      SendableRegistry.addLW(this, "PIDController", instances);
      P = super.getP();
      I = super.getI();
      D = super.getD();
      IZone = super.getIZone();
      FF = super.getFF();
    }

    public double getReference() {
      return setpoint;
    }

    @Override
    public REVLibError setReference(double value, ControlType control) {
      REVLibError ret = super.setReference(value, control);
      setpoint = value;
      type = control;
      stopped = false;
      return ret;
    }

    private double getCachedP() {
      return P;
    }

    private double getCachedI() {
      return I;
    }

    private double getCachedD() {
      return D;
    }

    private double getCachedFF() {
      return FF;
    }

    private double getCachedIZone() {
      return IZone;
    }

    @Override
    public REVLibError setP(double P) {
      REVLibError ret = super.setP(P);
      if (ret == REVLibError.kOk) {
        this.P = P;
      }
      return ret;
    }

    @Override
    public REVLibError setI(double I) {
      REVLibError ret = super.setI(I);
      if (ret == REVLibError.kOk) {
        this.I = I;
      }
      return ret;
    }

    @Override
    public REVLibError setD(double D) {
      REVLibError ret = super.setD(D);
      if (ret == REVLibError.kOk) {
        this.D = D;
      }
      return ret;
    }

    @Override
    public REVLibError setFF(double FF) {
      REVLibError ret = super.setFF(FF);
      if (ret == REVLibError.kOk) {
        this.FF = FF;
      }
      return ret;
    }

    @Override
    public REVLibError setIZone(double IZone) {
      REVLibError ret = super.setIZone(IZone);
      if (ret == REVLibError.kOk) {
        this.IZone = IZone;
      }
      return ret;
    }

    @Override
    public void initSendable(SendableBuilder builder) {

      // If there are issues with the SparkMAX, don't build widget
      if (clearFaults() != REVLibError.kOk) {
        return;
      }
      builder.setSmartDashboardType("PIDController");
      builder.addDoubleProperty("p", this::getCachedP, this::setP);
      builder.addDoubleProperty("i", this::getCachedI, this::setI);
      builder.addDoubleProperty("d", this::getCachedD, this::setD);
      builder.addDoubleProperty("ff", this::getCachedFF, this::setFF);
      builder.addDoubleProperty(
          "izone",
          this::getCachedIZone,
          (double toSet) -> {
            try {
              setIZone(toSet);
            } catch (IllegalArgumentException e) {
              MathSharedStore.reportError(
                  "IZone must be a non-negative number!", e.getStackTrace());
            }
          });
      builder.addDoubleProperty(
          "setpoint",
          this::getReference,
          (a) -> {
            setReference(a, type);
          });
    }
  }

  private class SparkRelativeEncoderSim extends SparkRelativeEncoder {

    SparkRelativeEncoderSim(CANSparkBase sparkMax, Type type, int countsPerRev) {
      super(sparkMax, type, countsPerRev);
    }

    @Override
    public double getPosition() {
      if (Robot.isReal()) {
        return super.getPosition();
      }
      return position;
    }

    @Override
    public REVLibError setPosition(double pos) {
      if (Robot.isReal()) {
        return super.setPosition(pos);
      }
      position = pos;
      return REVLibError.kOk;
    }

    @Override
    public double getVelocity() {
      if (Robot.isReal()) {
        return super.getVelocity();
      }
      return velocity;
    }
  }

  public static final double kPeriod = 0.001; // 1kHz
  private SparkPIDControllerSim ctrl;
  private final Object pidControllerLock = new Object();
  private SparkRelativeEncoderSim enc;
  private double prevError, IState, setpoint, position, velocity, output;
  private boolean stopped;
  private ControlType type;

  public CANSparkMaxSim(int deviceId, MotorType motorType) {
    super(deviceId, motorType);
    setpoint = 0;
    position = 0;
    velocity = 0;
    this.type = ControlType.kVelocity;
    IState = 0;
    output = 0;
    stopped = true;
  }

  @Override
  public RelativeEncoder getEncoder(SparkRelativeEncoder.Type encoderType, int countsPerRev) {
    if (Robot.isReal()) {
      return super.getEncoder(encoderType, countsPerRev);
    }
    if (enc == null) {
      enc = new SparkRelativeEncoderSim(this, encoderType, countsPerRev);
    }
    return enc;
  }

  @Override
  public SparkPIDControllerSim getPIDController() {
    throwIfClosed();
    synchronized (pidControllerLock) {
      if (ctrl == null) {
        ctrl = new SparkPIDControllerSim(this);
      }
      return ctrl;
    }
  }

  @Override
  public void stopMotor() {
    super.stopMotor();
    stopped = true;
    output = 0;
    IState = 0;
    prevError = 0;
  }

  public void update(double radsPerSec) {
    velocity = radsPerSec / (2 * Math.PI) * 60; // RPM
    position += radsPerSec / (2 * Math.PI) * kPeriod; // Revolutions
    calculate();
  }

  @Override
  public double get() {
    if (Robot.isReal()) {
      return super.get();
    }
    return output;
  }

  @Override
  public void set(double speed) {
    super.set(speed);
    output = speed;
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

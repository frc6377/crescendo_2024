// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.DebugEntry;

public class TurretSubsystem extends SubsystemBase {

  private CANSparkMax turretMotor;
  private PIDController turretPIDController;
  private CANcoder m_encoder;
  private double turretPosition;
  private double turretVelocity;
  private DebugEntry<Double> turretPositionEntry =
      new DebugEntry<Double>(turretPosition, "Position", this);
  private DebugEntry<Double> turretVelocityEntry =
      new DebugEntry<Double>(turretVelocity, "Velocity", this);

  public TurretSubsystem() {
    turretMotor = new CANSparkMax(Constants.TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(40);

    
    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 2);
    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -2);
    
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    
    // initialze PID controller and encoder objects
    turretPIDController =
        new PIDController(
            Constants.TurretConstants.TURRET_KP,
            Constants.TurretConstants.TURRET_KI,
            Constants.TurretConstants.TURRET_KD);
    m_encoder = new CANcoder(Constants.TurretConstants.CANcoder_ID);

    zeroTurretEncoder();
    turretPIDController.setIZone(Constants.TurretConstants.TURRET_KIZ);
  }

  public void stopTurret() {
    turretMotor.stopMotor();
  }

  public void setTurretPos(double setpoint) {
    turretMotor.set(MathUtil.clamp(turretPIDController.calculate(turretPosition, setpoint),0,220));
  }

  public void zeroTurretEncoder() {
    m_encoder.setPosition(0.0);
  }

  public void updateTurretPosition() {
    turretPosition =
        Math.toRadians(((m_encoder.getPosition().getValueAsDouble()) * 360)
            * Constants.TurretConstants.CONVERSION_FACTOR);
    setTurretPos(turretPosition);
    SmartDashboard.putNumber("Turret Position", turretPosition);
    SmartDashboard.putBoolean("Out of Bounds", Math.abs(turretPosition) > 3.14);
    SmartDashboard.putBoolean("Soft limit enabled forward", turretMotor.isSoftLimitEnabled(SoftLimitDirection.kForward));
    SmartDashboard.putBoolean("Soft limit enabled reverse", turretMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse));
  }

  public void lockOntoTag(double rotationFromTag) {
    turretMotor.set(turretPIDController.calculate(rotationFromTag, 0.0));
  }

  public double getTurretPos() {
    return turretPosition; // returns the absolute encoder position in radians
  }

  public double getTurretVel() {
    return turretVelocity;
  }

  public Command LockTurret() {
    return run(() -> lockOntoTag(Math.toRadians(LimelightHelpers.getTX(""))));
  }

  @Override
  public void periodic() {
    updateTurretPosition();
    turretVelocity =
        (m_encoder.getVelocity().getValueAsDouble())
            * 60; // changing from rotations per second to rotations per minute or rpm
    turretPositionEntry.log(turretPosition);
    turretVelocityEntry.log(turretVelocity);
  }

  public double turretFromOdometry(
      Pose2d robotPos) { // Doesn't work right now but will be used later
    return Math.atan(robotPos.getY() / robotPos.getX()) + robotPos.getRotation().getDegrees();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

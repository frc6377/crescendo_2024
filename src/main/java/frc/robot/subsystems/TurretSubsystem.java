// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.networktables.DebugEntry;

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

    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 1);
    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -1);

    // initialze PID controller and encoder objects
    turretPIDController =
        new PIDController(
            Constants.TurretConstants.TURRET_KP,
            Constants.TurretConstants.TURRET_KI,
            Constants.TurretConstants.TURRET_KD);
    m_encoder = new CANcoder(Constants.TurretConstants.CANcoder_ID);

    // set PID coefficients
    turretPIDController.setP(Constants.TurretConstants.TURRET_KP);
    turretPIDController.setI(Constants.TurretConstants.TURRET_KI);
    turretPIDController.setD(Constants.TurretConstants.TURRET_KD);
    turretPIDController.setIZone(Constants.TurretConstants.TURRET_KIZ);
  }

  public void stopTurret() {
    turretMotor.stopMotor();
  }

  public void setTurretPos(double setpoint) {
    turretMotor.set(turretPIDController.calculate(turretPosition, setpoint));
  }

  public void zeroTurretEncoder() {
    m_encoder.setPosition(0.0);
  }

  public void updateTurretPosition() {
    turretPosition =
        (Math.toRadians((m_encoder.getPosition().getValueAsDouble()) * 360))
            * Constants.TurretConstants.CONVERSION_FACTOR;
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
    return run(() -> lockOntoTag(LimelightHelpers.getTX("")));
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

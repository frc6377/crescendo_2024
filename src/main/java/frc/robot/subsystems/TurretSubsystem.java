// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.TunableNumber;
import java.util.function.Consumer;

public class TurretSubsystem extends SubsystemBase {

  private CANSparkMax turretMotor;
  private PIDController turretPIDController;
  private CANcoder m_encoder;
  private double turretPosition;
  private double turretVelocity;
  private Consumer<Double> testPosition;
  private DebugEntry<Double> turretPositionEntry =
      new DebugEntry<Double>(turretPosition, "Position", this);
  private DebugEntry<Double> turretVelocityEntry =
      new DebugEntry<Double>(turretVelocity, "Velocity", this);
  private TunableNumber tunableTestPosition =
      new TunableNumber("Turret Test Position", 110, testPosition);

  private final RobotStateManager robotStateManager;

  public TurretSubsystem(RobotStateManager robotStateManager) {
    turretMotor = new CANSparkMax(Constants.TurretConstants.MOTOR_ID, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(40);

    this.robotStateManager = robotStateManager;

    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 2);
    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -2);

    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    // initialze PID controller and encoder objects
    turretPIDController =
        new PIDController(
            Constants.TurretConstants.KP,
            Constants.TurretConstants.KI,
            Constants.TurretConstants.KD);
    m_encoder = new CANcoder(Constants.TurretConstants.CANcoder_ID);

    zeroTurretEncoder();
    turretPIDController.setIZone(Constants.TurretConstants.KIZ);
  }

  public void stopTurret() {
    turretMotor.stopMotor();
  }

  public Command stowTurret() {
    return new InstantCommand(() -> setTurretPos(Math.toRadians(110)));
  }

  public void setTurretPos(double setpoint) {
    turretMotor.set(
        MathUtil.clamp(
            turretPIDController.calculate(turretPosition, setpoint), 0, Math.toRadians(220)));
  }

  public void holdPosition() {
    setTurretPos(turretPosition);
  }

  public void zeroTurretEncoder() {
    m_encoder.setPosition(0.0);
  }

  public void updateTurretPosition() {
    turretPosition =
        Math.toRadians(
            ((m_encoder.getPosition().getValueAsDouble()) * 360)
                * Constants.TurretConstants.CONVERSION_FACTOR);
    SmartDashboard.putNumber("Turret Position", turretPosition);
    SmartDashboard.putBoolean("Out of Bounds", Math.abs(turretPosition) > 3.14);
    SmartDashboard.putBoolean(
        "Soft limit enabled forward", turretMotor.isSoftLimitEnabled(SoftLimitDirection.kForward));
    SmartDashboard.putBoolean(
        "Soft limit enabled reverse", turretMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse));
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

  public Command testTurretCommand() {
    return runEnd(() -> setTurretPos(Math.toRadians(tunableTestPosition.get())), this::stopTurret);
  }

  public void LockTurret() {
    turretPIDController.reset();
    lockOntoTag(Math.toRadians(LimelightHelpers.getTX("")));
  }

  public void TurretOdomCommand(Pose2d robotPos, Pose2d targetPos) {
    turretPIDController.reset();
    setTurretPos(getTurretRotationFromOdometry(robotPos, targetPos));
  }

  public Command buildTurretCommand(boolean limelightVisible, Pose2d robotPos, Pose2d targetPos) {
    return limelightVisible
        ? run(this::LockTurret)
        : runEnd(() -> TurretOdomCommand(robotPos, targetPos), this::stopTurret);
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

  public double getTurretRotationFromOdometry(
      Pose2d robotPos, Pose2d targetPos) { // Doesn't work right now but will be used later
    return Math.atan2(robotPos.getY() - targetPos.getY(), robotPos.getX() - targetPos.getX())
        + robotPos.getRotation().getDegrees();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

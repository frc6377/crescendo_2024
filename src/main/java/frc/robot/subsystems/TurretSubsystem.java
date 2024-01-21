// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private CANSparkMax turretMotor;
  private PIDController turretPIDController;
  private CANcoder m_encoder;
  private ShuffleboardTab turretTab = Shuffleboard.getTab("Turret");
  private GenericEntry turretPosition = turretTab.add("Position",0.0).getEntry();
  private GenericEntry turretVelocity = turretTab.add("Velocity", 0).getEntry();

  private double robotAngle = 90;
  private double robotXOffset = 10;
  private double robotYOffset = 10;

  public TurretSubsystem() {
    turretMotor = new CANSparkMax(Constants.TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(40);

    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 1);
    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -1);

    // initialze PID controller and encoder objects
    turretPIDController = new PIDController(Constants.TurretConstants.TURRET_KP, Constants.TurretConstants.TURRET_KI, Constants.TurretConstants.TURRET_KD);
    m_encoder = new CANcoder(Constants.TurretConstants.CANcoder_ID);

    // set PID coefficients
    turretPIDController.setP(Constants.TurretConstants.TURRET_KP);
    turretPIDController.setI(Constants.TurretConstants.TURRET_KI);
    turretPIDController.setD(Constants.TurretConstants.TURRET_KD);
    turretPIDController.setIZone(Constants.TurretConstants.TURRET_KIZ);
  }

  public void stopTurret(){
    turretMotor.stopMotor();
  }

  public void setTurretPos(double setpoint) {
    turretMotor.set(turretPIDController.calculate(getTurretPos(), setpoint));
  }

  public double getTurretPos(){
    return m_encoder.getPosition().getValueAsDouble(); //returns the absolute encoder position in degrees
  }

  public double getTurretVel(){
    return (m_encoder.getVelocity().getValueAsDouble())/6.0; //changing from degrees per second to revolutions per minute or rpm
  }

  public Command TurretCommand(){
    return runOnce(() -> turretMotor.set(turretPIDController.calculate(getTurretVel(),1)));
  }

  @Override
  public void periodic() {
    turretPosition.setDouble(getTurretPos()); 
    turretVelocity.setDouble(getTurretVel());
  }

  public double turretFromOdometry(Pose2d robotPos){
    return Math.atan(robotPos.getY()/robotPos.getX()) + robotPos.getRotation().getDegrees();
    // setTurretPos(setPoint);
  }

  public double magicMethod(Rotation2d limelightAngle){ //rename this method when it is created
    return limelightAngle.getDegrees();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

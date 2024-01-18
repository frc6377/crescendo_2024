// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private CANSparkMax turretMotor;
  private SparkPIDController turretPIDController;
  private RelativeEncoder m_encoder;

  private double robotAngle = 90;
  private double robotXOffset = 10;
private double robotYOffset = 10;

  public TurretSubsystem() {
    turretMotor = new CANSparkMax(Constants.TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(40);
    
    turretMotor.restoreFactoryDefaults();

    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 1);
    turretMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -1);

    // initialze PID controller and encoder objects
    turretPIDController = turretMotor.getPIDController();
    m_encoder = turretMotor.getEncoder();

    // set PID coefficients
    turretPIDController.setP(Constants.TurretConstants.TURRET_KP);
    turretPIDController.setI(Constants.TurretConstants.TURRET_KI);
    turretPIDController.setD(Constants.TurretConstants.TURRET_KD);
    turretPIDController.setIZone(Constants.TurretConstants.TURRET_KIZ);
    turretPIDController.setFF(Constants.TurretConstants.TURRET_KFF);
    turretPIDController.setOutputRange(Constants.TurretConstants.TURRET_KMINOUTPUT, Constants.TurretConstants.TURRET_KMAXOUTPUT);

    turretPIDController.setSmartMotionMaxVelocity(Constants.TurretConstants.TURRET_MAXVEL, Constants.TurretConstants.TURRET_SMARTMOTION_SLOT);
    turretPIDController.setSmartMotionMinOutputVelocity(Constants.TurretConstants.TURRET_MINVEL, Constants.TurretConstants.TURRET_SMARTMOTION_SLOT);
    turretPIDController.setSmartMotionMaxAccel(Constants.TurretConstants.TURRET_MAXACC, Constants.TurretConstants.TURRET_SMARTMOTION_SLOT);
    turretPIDController.setSmartMotionAllowedClosedLoopError(Constants.TurretConstants.TURRET_ALLOWEDERR, Constants.TurretConstants.TURRET_SMARTMOTION_SLOT);

  }

  public void setTurretPos(double pos) {
    turretPIDController.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setTurretVelo(double velo){
    turretPIDController.setReference(velo, CANSparkMax.ControlType.kVelocity);
  }

  public double getTurretPos(){
    return turretMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    
  }

  public double turretFromOdometry(Pose2d robotPos){
    return Math.atan(robotPos.getY()/robotPos.getX()) + robotPos.getRotation().getDegrees();
    // setTurretPos(setPoint);
  }

  public double magicMethod(Rotation2d limelightAngle){
    return limelightAngle.getDegrees();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

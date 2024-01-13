// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new TRShooterSubsystem. */
  private static final int deviceID = 3;
  private Boolean TLMotorBool1, TLMotorBool2, BRMotorBool1, BRMotorBool2;
  private CANSparkMax TLMotor1, TLMotor2, BRMotor1, BRMotor2;
  private double TLP, TLI, TLD, TLFF, TLIz, BRP, BRI, BRD, BRFF, BRIz;

  public double motorSpeed;

  public ShooterSubsystem() {
    // initialize motor
    if (TLMotorBool1) {
      TLMotor1 = new CANSparkMax(1, MotorType.kBrushless);
      TLMotor1.restoreFactoryDefaults();
    }
    if (TLMotorBool2) {
      TLMotor2 = new CANSparkMax(2, MotorType.kBrushless);
      TLMotor2.restoreFactoryDefaults();
      if 
      TLMotor2.follow(TLMotor1);
    }
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    // PID coefficients
    kP = 0.00012; 
    kI = 0.000001;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5500;
    motorSpeed = 2000;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Speed", motorSpeed);
  }

  public Command RunMotors() {
    return run(
        () -> {
          m_pidController.setReference(SmartDashboard.getNumber("Set Speed", 0), ControlType.kVelocity);
        });
  }

  public Command StopMotors() {
    return runOnce(
        () -> {
          m_motor.stopMotor();
          // BRmotor1.stopMotor();
        });
  }

  public Command RunFeeder() {
    return run(
        () -> {
          // feederMotor.set(feederVelo);
        });
  }

  public Command StopFeeder() {
    return runOnce(
        () -> {
          // feederMotor.stopMotor();
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    motorSpeed = SmartDashboard.getNumber("SetPoint", 0);
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    SmartDashboard.putNumber("RPM", m_motor.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

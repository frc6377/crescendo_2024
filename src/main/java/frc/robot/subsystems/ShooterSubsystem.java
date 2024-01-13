// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;
import com.fasterxml.jackson.databind.jsontype.BasicPolymorphicTypeValidator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  // Motors
  private Boolean TLMotorBool1, TLMotorBool2, BRMotorBool1, BRMotorBool2, feederBool;
  private CANSparkMax TLMotor1, TLMotor2, BRMotor1, BRMotor2, feederMotor;

  // PID Values
  private double TLP, TLI, TLD, TLFF, TLIz, BRP, BRI, BRD, BRFF, BRIz;

  // Motor IDs
  private int TLID1, TLID2, BRID1, BRID2, feederID;

  // Set Speeds
  public double TLMotorSpeed, BRMotorSpeed, feederSpeed;

  public ShooterSubsystem() {
    // Bools for if motor on bot
    TLMotorBool1 = true;
    TLMotorBool2 = true;
    BRMotorBool1 = true;
    BRMotorBool2 = true;
    feederBool = true;

    // IDs
    TLID1 = 1;
    TLID2 = 2;
    BRID1 = 3;
    BRID2 = 4;
    feederID = 5;

    // PID values
    TLP = 12e-3;
    TLI = 1e-5;
    TLD = 0;
    TLIz = 0;
    TLFF = 15e-4;

    BRP = 12e-3;
    BRI = 1e-5;
    BRD = 0;
    BRIz = 0;
    BRFF = 15e-4;

    // Motor Speeds
    TLMotorSpeed = 3000;
    BRMotorSpeed = 3000;
    feederSpeed = .5;

    // initialize motor
    if (TLMotorBool1) {
      TLMotor1 = new CANSparkMax(TLID1, MotorType.kBrushless);
      TLMotor1.restoreFactoryDefaults();
      TLMotor1.getPIDController().setP(TLP);
      TLMotor1.getPIDController().setI(TLI);
      TLMotor1.getPIDController().setD(TLD);
      TLMotor1.getPIDController().setIZone(TLIz);
      TLMotor1.getPIDController().setFF(TLFF);
    }
    if (TLMotorBool2) {
      TLMotor2 = new CANSparkMax(TLID2, MotorType.kBrushless);
      TLMotor2.restoreFactoryDefaults();
      TLMotor2.setInverted(true);
      if (TLMotorBool1) {
        TLMotor2.follow(TLMotor1);
      } else {
        TLMotor2.getPIDController().setP(TLP);
        TLMotor2.getPIDController().setI(TLI);
        TLMotor2.getPIDController().setD(TLD);
        TLMotor2.getPIDController().setIZone(TLIz);
        TLMotor2.getPIDController().setFF(TLFF);
      }
    }

    if (BRMotorBool1) {
      BRMotor1 = new CANSparkMax(BRID1, MotorType.kBrushless);
      BRMotor1.restoreFactoryDefaults();
      BRMotor1.getPIDController().setP(BRP);
      BRMotor1.getPIDController().setI(BRI);
      BRMotor1.getPIDController().setD(BRD);
      BRMotor1.getPIDController().setIZone(BRIz);
      BRMotor1.getPIDController().setFF(BRFF);
    }
    if (BRMotorBool2) {
      BRMotor2 = new CANSparkMax(BRID2, MotorType.kBrushless);
      BRMotor2.restoreFactoryDefaults();
      BRMotor2.setInverted(true);
      if (BRMotorBool1) {
        BRMotor2.follow(BRMotor1);
      } else {
        BRMotor2.getPIDController().setP(BRP);
        BRMotor2.getPIDController().setI(BRI);
        BRMotor2.getPIDController().setD(BRD);
        BRMotor2.getPIDController().setIZone(BRIz);
        BRMotor2.getPIDController().setFF(BRFF);
      }
    }

    if (feederBool) {
      feederMotor = new CANSparkMax(feederID, MotorType.kBrushless);
      feederMotor.restoreFactoryDefaults();
    }

    // Shuffle Board
    Shooter_SBTab = new ShuffleBoard
  }

  public Command RunMotors() {
    return run(
        () -> {
          if (TLMotorBool1) { TLMotor1.getPIDController().setReference(TLMotorSpeed, ControlType.kVelocity); }
          else if (TLMotorBool2) { TLMotor2.getPIDController().setReference(TLMotorSpeed, ControlType.kVelocity); }
          if (BRMotorBool1) { BRMotor1.getPIDController().setReference(BRMotorSpeed, ControlType.kVelocity); }
          else if (BRMotorBool2) { BRMotor2.getPIDController().setReference(BRMotorSpeed, ControlType.kVelocity); }
        });
  }

  public Command StopMotors() {
    return runOnce(
        () -> {
          TLMotor1.stopMotor();
          TLMotor2.stopMotor();
          BRMotor1.stopMotor();
          BRMotor2.stopMotor();
        });
  }

  public Command RunFeeder() {
    return run(
        () -> {
          feederMotor.set(feederSpeed);
        });
  }

  public Command StopFeeder() {
    return runOnce(
        () -> {
          feederMotor.stopMotor();
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
    // motorSpeed = SmartDashboard.getNumber("SetPoint", 0);
    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { m_pidController.setP(p); kP = p; }
    // if((i != kI)) { m_pidController.setI(i); kI = i; }
    // if((d != kD)) { m_pidController.setD(d); kD = d; }
    // if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   m_pidController.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 
    }

    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    SmartDashboard.putNumber("RPM", m_motor.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

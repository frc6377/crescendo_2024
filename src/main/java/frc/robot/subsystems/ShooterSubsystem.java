// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

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
    TLMotorBool2 = false;
    BRMotorBool1 = false;
    BRMotorBool2 = false;
    feederBool = false;

    // IDs
    TLID1 = 3;
    TLID2 = 2;
    BRID1 = 3;
    BRID2 = 4;
    feederID = 5;

    // PID values
    TLP = 12e-5;
    TLI = 1e-6;
    TLD = 0;
    TLIz = 0;
    TLFF = 15e-6;

    BRP = 12e-5;
    BRI = 1e-6;
    BRD = 0;
    BRIz = 0;
    BRFF = 15e-6;

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

    // SmartDashboard
    if (TLMotorBool1 || TLMotorBool2) {
      SmartDashboard.putNumber("Top/Left P", TLP);
      SmartDashboard.putNumber("Top/Left I", TLI);
      SmartDashboard.putNumber("Top/Left D", TLD);
      SmartDashboard.putNumber("Top/Left FF", TLFF);
    }
    if (BRMotorBool1 || BRMotorBool2) {
      SmartDashboard.putNumber("Bottom/Right P", BRP);
      SmartDashboard.putNumber("Bottom/Right I", BRI);
      SmartDashboard.putNumber("Bottom/Right D", BRD);
      SmartDashboard.putNumber("Bottom/Right FF", BRFF);
    }
    SmartDashboard.putNumber("TL Set Speed 1", TLMotorSpeed);

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
          if (TLMotorBool1) { TLMotor1.stopMotor(); }
          if (TLMotorBool2) { TLMotor2.stopMotor(); }
          if (BRMotorBool1) { BRMotor1.stopMotor(); }
          if (BRMotorBool2) { BRMotor2.stopMotor(); }
        });
  }

  public Command RunFeeder() {
    return run(
        () -> {
          if (feederBool) { feederMotor.set(feederSpeed); }
        });
  }

  public Command StopFeeder() {
    return runOnce(
        () -> {
          if (feederBool) { feederMotor.stopMotor(); }
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
    if (TLMotorBool1) { SmartDashboard.putNumber("TL RPM1", TLMotor1.getEncoder().getVelocity()); }
    if (TLMotorBool2) { SmartDashboard.putNumber("TL RPM2", TLMotor2.getEncoder().getVelocity()); }
    if (BRMotorBool1) { SmartDashboard.putNumber("BR RPM1", BRMotor1.getEncoder().getVelocity()); }
    if (BRMotorBool2) { SmartDashboard.putNumber("BR RPM2", BRMotor2.getEncoder().getVelocity()); }
    if (feederBool) { feederSpeed = SmartDashboard.getNumber("Feeder Speed", feederSpeed); }

    if (TLMotorBool1 == true || TLMotorBool2 == true) {
      TLP = SmartDashboard.getNumber("Top/Left P", TLP);
      TLI = SmartDashboard.getNumber("Top/Left I", TLI);
      TLD = SmartDashboard.getNumber("Top/Left D", TLD);
      TLFF = SmartDashboard.getNumber("Top/Left FF", TLFF);
      TLMotorSpeed = SmartDashboard.getNumber("TL Set Speed", TLMotorSpeed);
    }

    if (BRMotorBool1 || BRMotorBool2) {
      BRP = SmartDashboard.getNumber("Bottom/Right P", BRP);
      BRI = SmartDashboard.getNumber("Bottom/Right I", BRI);
      BRD = SmartDashboard.getNumber("Bottom/Right D", BRD);
      BRFF = SmartDashboard.getNumber("Bottom/Right FF", BRFF);
      BRMotorSpeed = SmartDashboard.getNumber("BR Set Speed", BRMotorSpeed);
    }

    SmartDashboard.putNumber("Set Pose", TL);
  }

  @Override
  public void simulationPeriodic() {
    
  }
}

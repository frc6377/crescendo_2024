// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  // Motors
  private Boolean leftEnabled, rightEnabled, BRMotorBool1, BRMotorBool2, feederBool1, feederBool2;
  private CANSparkMax leftMotor, rightMotor, BRMotor1, BRMotor2, feederMotor1, feederMotor2;

  // PID Values
  private double TLP1, TLI1, TLD1, TLFF1, TLIz1, TLP2, TLI2, TLD2, TLFF2, TLIz2;
  private double BRP, BRI, BRD, BRFF, BRIz;

  // Motor IDs
  private int TLID1, TLID2, BRID1, BRID2, feederID1, feederID2;

  // Set Speeds
  public double leftMotorSpeed, rightMotorSpeed, feederSpeed;

  public ShooterSubsystem() {
    // Bools for if motor on bot
    leftEnabled = true;
    rightEnabled = true;
    BRMotorBool1 = false;
    BRMotorBool2 = false;
    feederBool1 = false;
    feederBool2 = false;

    // IDs
    TLID1 = 1;
    TLID2 = 4;
    BRID1 = 3;
    BRID2 = 2;
    feederID1 = 1;
    feederID2 = 1;

    // PID values
    TLP1 = 36e-5;
    TLI1 = 5e-7;
    TLD1 = 1e-4;
    TLIz1 = 0;
    TLFF1 = 2e-6;

    TLP2 = 36e-5;
    TLI2 = 5e-7;
    TLD2 = 1e-4;
    TLIz2 = 0;
    TLFF2 = 2e-6;

    BRP = 12e-5;
    BRI = 1e-6;
    BRD = 0;
    BRIz = 0;
    BRFF = 15e-6;

    // Motor Speeds
    leftMotorSpeed = 2700;
    rightMotorSpeed = 1000;
    feederSpeed = 100;

    // initialize LT motor
    if (leftEnabled) {
      leftMotor = new CANSparkMax(TLID1, MotorType.kBrushless);
      leftMotor.restoreFactoryDefaults();
      leftMotor.getPIDController().setP(TLP1);
      leftMotor.getPIDController().setI(TLI1);
      leftMotor.getPIDController().setD(TLD1);
      leftMotor.getPIDController().setIZone(TLIz1);
      leftMotor.getPIDController().setFF(TLFF1);
    }

    // initialize TL motor 2
    if (rightEnabled) {
      rightMotor = new CANSparkMax(TLID2, MotorType.kBrushless);
      rightMotor.restoreFactoryDefaults();
      rightMotor.setInverted(false);
      rightMotor.getPIDController().setP(TLP1);
      rightMotor.getPIDController().setI(TLI1);
      rightMotor.getPIDController().setD(TLD1);
      rightMotor.getPIDController().setIZone(TLIz1);
      rightMotor.getPIDController().setFF(TLFF1);
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

    if (feederBool1) {
      feederMotor1 = new CANSparkMax(feederID1, MotorType.kBrushless);
      feederMotor1.restoreFactoryDefaults();
    }
    if (feederBool2) {
      feederMotor2 = new CANSparkMax(feederID2, MotorType.kBrushless);
      feederMotor2.restoreFactoryDefaults();
      feederMotor2.setInverted(true);
    }

    // SmartDashboard
    if (leftEnabled) {
      Preferences.initDouble("P", TLP1);
      Preferences.initDouble("I", TLI1);
      Preferences.initDouble("D", TLD1);
      Preferences.initDouble("FF", TLFF1);
    }
    // if (TLMotorBool2) {
    //   SmartDashboard.putNumber("Top/Left P 2", TLP2);
    //   SmartDashboard.putNumber("Top/Left I 2", TLI2);
    //   SmartDashboard.putNumber("Top/Left D 2", TLD2);
    //   SmartDashboard.putNumber("Top/Left FF 2", TLFF2);
    // }
    if (leftEnabled || rightEnabled) {
      Preferences.initDouble("Left Set Speed", leftMotorSpeed);
      Preferences.initDouble("Right Set Speed", rightMotorSpeed);
    }
    if (BRMotorBool1 || BRMotorBool2) {
      SmartDashboard.putNumber("Bottom/Right P", BRP);
      SmartDashboard.putNumber("Bottom/Right I", BRI);
      SmartDashboard.putNumber("Bottom/Right D", BRD);
      SmartDashboard.putNumber("Bottom/Right FF", BRFF);
    }

    if (feederBool1 || feederBool2) {
      SmartDashboard.putNumber("Feeder Speed", feederSpeed);
    }
  }

  public Command RunMotors() {
    return run(
        () -> {
          if (leftEnabled) {
            leftMotor.getPIDController().setReference(leftMotorSpeed, ControlType.kVelocity);
          }
          if (rightEnabled) {
            rightMotor.getPIDController().setReference(rightMotorSpeed, ControlType.kVelocity);
          }
          if (BRMotorBool1) {
            BRMotor1.getPIDController().setReference(rightMotorSpeed, ControlType.kVelocity);
          }
          if (BRMotorBool2) {
            BRMotor2.getPIDController().setReference(rightMotorSpeed, ControlType.kVelocity);
          }
        });
  }

  public Command StopMotors() {
    return runOnce(
        () -> {
          if (leftEnabled) {
            leftMotor.stopMotor();
          }
          if (rightEnabled) {
            rightMotor.stopMotor();
          }
          if (BRMotorBool1) {
            BRMotor1.stopMotor();
          }
          if (BRMotorBool2) {
            BRMotor2.stopMotor();
          }
        });
  }

  public Command RunFeeder() {
    return run(
        () -> {
          if (feederBool1) {
            feederMotor1.set(feederSpeed);
          }
          if (feederBool2) {
            feederMotor2.set(feederSpeed);
          }
        });
  }

  public Command StopFeeder() {
    return runOnce(
        () -> {
          if (feederBool1) {
            feederMotor1.stopMotor();
          }
          if (feederBool2) {
            feederMotor2.stopMotor();
          }
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
    if (leftEnabled) {
      SmartDashboard.putNumber("Left RPM", leftMotor.getEncoder().getVelocity());
    }
    if (rightEnabled) {
      SmartDashboard.putNumber("Right RPM", rightMotor.getEncoder().getVelocity());
    }
    if (BRMotorBool1) {
      SmartDashboard.putNumber("BR RPM1", BRMotor1.getEncoder().getVelocity());
    }
    if (BRMotorBool2) {
      SmartDashboard.putNumber("BR RPM2", BRMotor2.getEncoder().getVelocity());
    }
    if (feederBool1 || feederBool2) {
      feederSpeed = SmartDashboard.getNumber("Feeder Speed", feederSpeed);
    }

    if (leftEnabled || rightEnabled) {
      TLP1 = Preferences.getDouble("P", TLP1);
      TLI1 = Preferences.getDouble("I", TLI1);
      TLD1 = Preferences.getDouble("D", TLD1);
      TLFF1 = Preferences.getDouble("FF", TLFF1);
      leftMotor.getPIDController().setI(TLI1);
      leftMotor.getPIDController().setD(TLD1);
      leftMotor.getPIDController().setP(TLP1);
      leftMotor.getPIDController().setFF(TLFF1);
      rightMotor.getPIDController().setI(TLI1);
      rightMotor.getPIDController().setD(TLD1);
      rightMotor.getPIDController().setP(TLP1);
      rightMotor.getPIDController().setFF(TLFF1);
      leftMotorSpeed = Preferences.getDouble("Left Set Speed", 0);
      rightMotorSpeed = Preferences.getDouble("Right Set Speed", 0);
    }

    if (BRMotorBool1 || BRMotorBool2) {
      BRP = SmartDashboard.getNumber("Bottom/Right P", BRP);
      BRI = SmartDashboard.getNumber("Bottom/Right I", BRI);
      BRD = SmartDashboard.getNumber("Bottom/Right D", BRD);
      BRFF = SmartDashboard.getNumber("Bottom/Right FF", BRFF);
      BRMotor1.getPIDController().setP(BRP);
      BRMotor1.getPIDController().setI(BRI);
      BRMotor1.getPIDController().setD(BRD);
      BRMotor1.getPIDController().setFF(BRFF);
    }
  }

  @Override
  public void simulationPeriodic() {}
}

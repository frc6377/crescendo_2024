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
  /** Creates a new TRShooterSubsystem. */
  private double shooterVelo;

  private double TLP;
  private double TLI;
  private double TLD;

  private double BRP;
  private double BRI;
  private double BRD;

  private double feederVelo;

  private CANSparkMax TLmotor1;
  private CANSparkMax TLmotor2;
  private CANSparkMax BRmotor1;
  private CANSparkMax BRmotor2;
  private CANSparkMax feederMotor;

  public ShooterSubsystem() {
    TLP = 1e-6;
    TLI = 0;
    TLD = 0;

    BRP = 1e-6;
    BRI = 0;
    BRD = 0;

    shooterVelo = 0.5;
    feederVelo = 0.25;

    // TL = Top / Left
    TLmotor1 = new CANSparkMax(1, MotorType.kBrushless);
    TLmotor1.getPIDController().setP(TLP);
    TLmotor1.getPIDController().setI(TLI);
    TLmotor1.getPIDController().setD(TLD);
    TLmotor2 = new CANSparkMax(2, MotorType.kBrushless);
    TLmotor2.follow(TLmotor1);

    // BR = Bottom / Right
    BRmotor1 = new CANSparkMax(3, MotorType.kBrushless);
    BRmotor1.setInverted(true);
    BRmotor1.getPIDController().setP(BRP);
    BRmotor1.getPIDController().setI(BRI);
    BRmotor1.getPIDController().setD(BRD);
    BRmotor2 = new CANSparkMax(4, MotorType.kBrushless);
    BRmotor2.follow(BRmotor1);

    feederMotor = new CANSparkMax(5, MotorType.kBrushless);

    SmartDashboard.putNumber("shooterVelo", shooterVelo);
    SmartDashboard.putNumber("feederVelo", feederVelo);

    SmartDashboard.putNumber("TLShooterP", TLP);
    SmartDashboard.putNumber("TLShooterI", TLI);
    SmartDashboard.putNumber("TLShooterD", TLD);

    SmartDashboard.putNumber("BRShooterP", BRP);
    SmartDashboard.putNumber("BRShooterI", BRI);
    SmartDashboard.putNumber("BRShooterD", BRD);
  }

  public Command RunMotors() {
    return run(
        () -> {
          TLmotor1.getPIDController().setReference(shooterVelo, ControlType.kVelocity);
          BRmotor1.getPIDController().setReference(shooterVelo, ControlType.kVelocity);
        });
  }

  public Command StopMotors() {
    return runOnce(
        () -> {
          TLmotor1.stopMotor();
          BRmotor1.stopMotor();
        });
  }

  public Command adjustSpeed(double testSpeedOffset) {
    return run(
        () -> {
          shooterVelo += testSpeedOffset;
        });
  }

  public Command RunFeeder() {
    return run(
        () -> {
          feederMotor.set(feederVelo);
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
    shooterVelo = SmartDashboard.getNumber("shooterVelo", shooterVelo);
    feederVelo = SmartDashboard.getNumber("feederVelo", feederVelo);
    adjustTLP();
    adjustTLI();
    adjustTLD();

    adjustBRP();
    adjustBRI();
    adjustBRD();
  }

  public void adjustTLP() {
    double get_TLP_num = SmartDashboard.getNumber("TLShooterP", TLP);
    if (TLP != get_TLP_num) {
      TLP = get_TLP_num;
      TLmotor1.getPIDController().setP(TLP);
    }
  }

  public void adjustTLI() {
    double get_TLI_num = SmartDashboard.getNumber("TLShooterI", TLI);
    if (TLI != get_TLI_num) {
      TLI = get_TLI_num;
      TLmotor1.getPIDController().setI(TLI);
    }
  }

  public void adjustTLD() {
    double get_TLD_num = SmartDashboard.getNumber("TLShooterD", TLD);
    if (TLD != get_TLD_num) {
      TLD = get_TLD_num;
      TLmotor1.getPIDController().setD(TLD);
    }
  }

  public void adjustBRP() {
    double get_BRP_num = SmartDashboard.getNumber("BRShooterP", BRP);
    if (BRP != get_BRP_num) {
      BRP = get_BRP_num;
      BRmotor1.getPIDController().setP(BRP);
    }
  }

  public void adjustBRI() {
    double get_BRI_num = SmartDashboard.getNumber("BRShooterI", BRI);
    if (BRI != get_BRI_num) {
      BRI = get_BRI_num;
      BRmotor1.getPIDController().setI(BRI);
    }
  }

  public void adjustBRD() {
    double get_BRD_num = SmartDashboard.getNumber("BRShooterD", BRD);
    if (BRD != get_BRD_num) {
      BRD = get_BRD_num;
      BRmotor1.getPIDController().setD(BRD);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

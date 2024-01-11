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
    TLP = 1.0;
    TLI = 0;
    TLD = 0;

    BRP = 1.0;
    BRI = 0;
    BRD = 0;

    shooterVelo = 0.5;
    feederVelo = 0.25;

    TLmotor1 = new CANSparkMax(1, MotorType.kBrushless);
    TLmotor1.getPIDController().setP(TLP);
    TLmotor1.getPIDController().setI(TLI);
    TLmotor1.getPIDController().setD(TLD);
    TLmotor2 = new CANSparkMax(2, MotorType.kBrushless);
    TLmotor2.follow(TLmotor1);

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

    SmartDashboard.putNumber("TRShooterP", TLP);
    SmartDashboard.putNumber("TRShooterI", TLI);
    SmartDashboard.putNumber("TRShooterD", TLD);

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
        shooterVelo+=testSpeedOffset;
      }
    );
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
    if(TLP != SmartDashboard.getNumber("TRShooterP", TLP)){
      adjustTLP();
    }
    if(TLI != SmartDashboard.getNumber("TRShooterI", TLI)){
      adjustTLI();
    }
    if(TLD != SmartDashboard.getNumber("TRShooterD", TLD)){
      adjustTLD();
    }

    if(BRP != SmartDashboard.getNumber("BRShooterP", BRP)){
        adjustBRP();
    }
    if(BRI != SmartDashboard.getNumber("BRShooterI", BRI)){
        adjustBRI();
    }
    if(BRD != SmartDashboard.getNumber("BRShooterD", BRD)){
        adjustBRD();
    }

    SmartDashboard.putNumber("shooterVelo", shooterVelo);
  }

  public void adjustTLP(){
    TLP = SmartDashboard.getNumber("TRShooterP", TLP);
    TLmotor1.getPIDController().setP(TLP);
    SmartDashboard.putNumber("TRPShooterP", TLP);
  }

  public void adjustTLI(){
    TLI = SmartDashboard.getNumber("TRShooterI", TLI);
    TLmotor1.getPIDController().setI(TLI);
    SmartDashboard.putNumber("TRShooterI", TLI);
  }

  public void adjustTLD(){
    TLD = SmartDashboard.getNumber("TRShooterD", TLD);
    TLmotor1.getPIDController().setD(TLD);
    SmartDashboard.putNumber("TRShooterD", TLD);
  }

  public void adjustBRP(){
    BRP = SmartDashboard.getNumber("BRShooterP", BRP);
    BRmotor1.getPIDController().setP(BRP);
    SmartDashboard.putNumber("BRPShooterP", BRP);
  }

  public void adjustBRI(){
      BRI = SmartDashboard.getNumber("BRShooterI", BRI);
      BRmotor1.getPIDController().setI(BRI);
      SmartDashboard.putNumber("BRShooterI", BRI);
  }

  public void adjustBRD(){
      BRD = SmartDashboard.getNumber("BRShooterD", BRD);
      BRmotor1.getPIDController().setD(BRD);
      SmartDashboard.putNumber("BRShooterD", BRD);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

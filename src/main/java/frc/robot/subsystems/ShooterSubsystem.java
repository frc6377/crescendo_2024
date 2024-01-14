// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new TRShooterSubsystem. */
  private double shooterVelo;
  private double shooterVelo2;

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
  private CANSparkMax feederMotor2;

  public ShooterSubsystem() {
    TLP = 1e-4;
    TLI = 0;
    TLD = 0;

    BRP = 1e-4;
    BRI = 0;
    BRD = 0;

    shooterVelo = .2;
    // shooterVelo2 = .60;
    feederVelo = -.5;

    // TL = Top / Left
    TLmotor1 = new CANSparkMax(5, MotorType.kBrushless);
    TLmotor1.restoreFactoryDefaults();
    TLmotor1.getPIDController().setP(TLP);
    TLmotor1.getPIDController().setI(TLI);
    TLmotor1.getPIDController().setD(TLD);
    TLmotor2 = new CANSparkMax(4, MotorType.kBrushless);
    TLmotor2.restoreFactoryDefaults();
    // TLmotor2.setInverted(true);
    TLmotor2.follow(TLmotor1);

    // BR = Bottom / Right
    // BRmotor1 = new CANSparkMax(4, MotorType.kBrushless);
    // BRmotor1.setInverted(true);
    // BRmotor1.getPIDController().setP(BRP);
    // BRmotor1.getPIDController().setI(BRI);
    // BRmotor1.getPIDController().setD(BRD);
    // BRmotor2 = new CANSparkMax(1, MotorType.kBrushless);
    // BRmotor2.follow(BRmotor1);

    feederMotor = new CANSparkMax(1, MotorType.kBrushless);
    feederMotor.restoreFactoryDefaults();
    // feederMotor2 = new CANSparkMax(2, MotorType.kBrushless);
    // feederMotor2.restoreFactoryDefaults();
    // feederMotor2.setInverted(true);

    SmartDashboard.putNumber("Shooter Velocity", shooterVelo);
    // SmartDashboard.putNumber("Shooter Velocity 2", shooterVelo2);
    SmartDashboard.putNumber("Feeder Velocity", feederVelo);

    SmartDashboard.putNumber("Top/Left P", TLP);
    SmartDashboard.putNumber("Top/Left I", TLI);
    SmartDashboard.putNumber("Top/Left D", TLD);

    SmartDashboard.putNumber("Bottom/Right P", BRP);
    SmartDashboard.putNumber("Bottom/Right I", BRI);
    SmartDashboard.putNumber("Bottom/Right D", BRD);
  }

  public Command RunMotors() {
    return new InstantCommand(
        () -> {
          TLmotor1.getPIDController().setReference(shooterVelo, ControlType.kDutyCycle);
          // BRmotor1.getPIDController().setReference(shooterVelo, ControlType.kDutyCycle);
        });
  }

  public Command StopMotors() {
    return runOnce(
        () -> {
          TLmotor1.stopMotor();
          // TLmotor2.stopMotor();
          // BRmotor1.stopMotor();
        });
  }

  public Command RunFeeder() {
    return run(
        () -> {
          feederMotor.set(feederVelo);
          // feederMotor2.set(feederVelo);
        });
  }

  public Command StopFeeder() {
    return runOnce(
        () -> {
          feederMotor.stopMotor();
          // feederMotor2.stopMotor();
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
    shooterVelo = SmartDashboard.getNumber("Shooter Velocity", shooterVelo);
    // shooterVelo2 = SmartDashboard.getNumber("Shooter Velocity 2", shooterVelo2);
    feederVelo = SmartDashboard.getNumber("Feeder Velocity", feederVelo);
    adjustTLP();
    adjustTLI();
    adjustTLD();
    SmartDashboard.putData(this);
    // adjustBRP();
    // adjustBRI();
    // adjustBRD();

    SmartDashboard.putNumber("Percent Output", TLmotor1.getAppliedOutput());
    SmartDashboard.putNumber("RPM", TLmotor1.getEncoder().getVelocity());
    SmartDashboard.putNumber("RPM2", TLmotor2.getEncoder().getVelocity());
  }

  public void adjustTLP() {
    double get_TLP_num = SmartDashboard.getNumber("Top/Left P", TLP);
    if (TLP != get_TLP_num) {
      TLP = get_TLP_num;
      TLmotor1.getPIDController().setP(TLP);
    }
  }

  public void adjustTLI() {
    double get_TLI_num = SmartDashboard.getNumber("Top/Left I", TLI);
    if (TLI != get_TLI_num) {
      TLI = get_TLI_num;
      TLmotor1.getPIDController().setI(TLI);
    }
  }

  public void adjustTLD() {
    double get_TLD_num = SmartDashboard.getNumber("Top/Left D", TLD);
    if (TLD != get_TLD_num) {
      TLD = get_TLD_num;
      TLmotor1.getPIDController().setD(TLD);
    }
  }

  // public void adjustBRP() {
  //   double get_BRP_num = SmartDashboard.getNumber("Bottom/Right P", BRP);
  //   if (BRP != get_BRP_num) {
  //     BRP = get_BRP_num;
  //     BRmotor1.getPIDController().setP(BRP);
  //   }
  // }

  // public void adjustBRI() {
  //   double get_BRI_num = SmartDashboard.getNumber("Bottom/Right I", BRI);
  //   if (BRI != get_BRI_num) {
  //     BRI = get_BRI_num;
  //     BRmotor1.getPIDController().setI(BRI);
  //   }
  // }

  // public void adjustBRD() {
  //   double get_BRD_num = SmartDashboard.getNumber("Bottom/Right D", BRD);
  //   if (BRD != get_BRD_num) {
  //     BRD = get_BRD_num;
  //     BRmotor1.getPIDController().setD(BRD);
  //   }
  // }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

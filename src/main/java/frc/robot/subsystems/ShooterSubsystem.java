// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private double setPoint;

  private double TLP;
  private double TLI;
  private double TLD;

  private double BRP;
  private double BRI;
  private double BRD;

  private double feederSetPoint;

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

    setPoint = 0.5;
    feederSetPoint = 0.25;

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
  }

  public Command RunMotors() {
    return run(
        () -> {
          TLmotor1.getPIDController().setReference(setPoint, ControlType.kVelocity);
          BRmotor1.getPIDController().setReference(setPoint, ControlType.kVelocity);
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
        setPoint+=testSpeedOffset;
      }
    );
  }

  public Command RunFeeder() {
    return run(
        () -> {
          feederMotor.set(feederSetPoint);
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

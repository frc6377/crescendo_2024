// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private double P;

  private double I;
  private double D;
  private double setPoint;
  private double feederSetPoint;
  private PIDController pid;

  private CANSparkMax TLmotor1;
  private CANSparkMax TLmotor2;
  private CANSparkMax BRmotor1;
  private CANSparkMax BRmotor2;
  private CANSparkMax feederMotor;

  public ShooterSubsystem() {
    P = 1.0;
    I = 1.0;
    D = 1.0;
    setPoint = 0.5;
    feederSetPoint = 0.25;

    TLmotor1 = new CANSparkMax(1, MotorType.kBrushless);
    TLmotor1.getPIDController().setP(P);
    TLmotor1.getPIDController().setI(I);
    TLmotor1.getPIDController().setD(D);
    TLmotor2 = new CANSparkMax(2, MotorType.kBrushless);
    TLmotor2.follow(TLmotor1);

    BRmotor1 = new CANSparkMax(3, MotorType.kBrushless);
    BRmotor1.setInverted(true);
    BRmotor1.getPIDController().setP(P);
    BRmotor1.getPIDController().setI(I);
    BRmotor1.getPIDController().setD(D);
    BRmotor2 = new CANSparkMax(4, MotorType.kBrushless);
    BRmotor2.follow(BRmotor1);

    feederMotor = new CANSparkMax(5, MotorType.kBrushless);
  }

  public Command RunMotors() {
    return run(
        () -> {
          TLmotor1.set(pid.calculate(TLmotor1.get(), setPoint));
          BRmotor1.set(pid.calculate(TLmotor1.get(), setPoint));
        });
  }

  public Command StopMotors() {
    return runOnce(
        () -> {
          TLmotor1.stopMotor();
          BRmotor1.stopMotor();
        });
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

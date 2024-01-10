// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private PIDController pid;


  private Spark TLmotor1;
  private Spark TLmotor2;
  private Spark BRmotor1;
  private Spark BRmotor2;
  private Spark otherMotor;

  public ShooterSubsystem() {
    pid = new PIDController(1, 1, 1);
    //pid.setIntegratorRange(-1, 1);

    TLmotor1 = new Spark(1);
    TLmotor2 = new Spark(2);
    TLmotor2.addFollower(TLmotor1);

    BRmotor1 = new Spark(3);
    BRmotor1.setInverted(true);
    BRmotor2 = new Spark(4);
    BRmotor2.addFollower(BRmotor1);

    otherMotor = new Spark(5);
  }

  public Command RunMotors() {
    return run(
        () -> {
          TLmotor1.set(pid.calculate(TLmotor1.get(), .5));
          TLmotor2.set(pid.calculate(TLmotor1.get(), .5));
          BRmotor1.set(pid.calculate(TLmotor1.get(), .5));
          BRmotor2.set(pid.calculate(TLmotor1.get(), .5));
          otherMotor.set(pid.calculate(otherMotor.get(), .5));
        });
  }

  public Command StopMotors() {
    return runOnce(
        () -> {
          TLmotor1.stopMotor();
          TLmotor2.stopMotor();
          BRmotor1.stopMotor();
          BRmotor2.stopMotor();
          otherMotor.stopMotor();
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

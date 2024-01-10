// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private Spark TLmotor1;
  private Spark TLmotor2;
  private Spark BRmotor1;
  private Spark BRmotor2;

  public ShooterSubsystem() {
    TLmotor1 = new Spark(1);
    TLmotor2 = new Spark(2);
    TLmotor2.addFollower(TLmotor1);

    BRmotor1 = new Spark(3);
    BRmotor2 = new Spark(4);
    BRmotor2.addFollower(BRmotor1);
  }

  public Command RunMotors() {
    return run(
        () -> {
          TLmotor1.set(1);
          TLmotor2.set(1);
          BRmotor1.set(1);
          BRmotor2.set(1);
        });
  }

  public Command StopMotors() {
    return run(
        () -> {
          TLmotor1.set(0);
          TLmotor2.set(0);
          BRmotor1.set(0);
          BRmotor2.set(0);
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

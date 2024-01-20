// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private CANSparkMax chooserMotor;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    chooserMotor =
        new CANSparkMax(Constants.IntakeConstants.INTAKE_CHOOSER_ID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    chooserMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(40);
    chooserMotor.setSmartCurrentLimit(20);
  }

  public Command intakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  // TODO: Add check to make sure turret is below 45 degrees before running & add photogate when
  // implemented.
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void reverseIntake(double speed) {
    intakeMotor.set(-speed);
  }

  public void setChooserSpeed(double speed) {
    chooserMotor.set(speed);
  }

  public void reverseChooser(double speed) {
    chooserMotor.set(-speed);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

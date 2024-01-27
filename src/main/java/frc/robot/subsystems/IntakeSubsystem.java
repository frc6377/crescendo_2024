// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.stateManagement.RobotStateManager;

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

  // TODO: Add check to make sure turret is below 45 degrees before running & add photogate when
  // implemented.
  public void runIntake() {
    intakeMotor.set(Constants.IntakeConstants.INTAKE_PERCENTAGE);
  }

  public void reverseIntake() {
    intakeMotor.set(-Constants.IntakeConstants.INTAKE_PERCENTAGE);
  }

  public void runChooser() {
    chooserMotor.set(Constants.IntakeConstants.INTAKE_PERCENTAGE);
  }

  public void reverseChooser() {
    chooserMotor.set(-Constants.IntakeConstants.INTAKE_PERCENTAGE);
  }

  public void speakerIntake() {
    runIntake();
    runChooser();
  }

  public void trapIntake() {
    runIntake();
    reverseChooser();
  }

  public void stopMotors() {
    chooserMotor.set(0);
    intakeMotor.set(0);
  }

  public Command intakeCommand(RobotStateManager robotStateManager) {
    return robotStateManager.getPlacementMode().getAsInt() == 0
        ? runOnce(() -> speakerIntake())
        : runOnce(() -> trapIntake());
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

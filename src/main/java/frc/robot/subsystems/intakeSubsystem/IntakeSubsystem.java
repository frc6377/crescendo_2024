// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utilities.DebugEntry;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMotor;
  private CANSparkMax chooserMotor;
  private DebugEntry<Double> intakeOutput;
  private DebugEntry<Double> chooserOutput;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID, "rio");
    chooserMotor =
        new CANSparkMax(
            Constants.IntakeConstants.INTAKE_CHOOSER_ID, MotorType.kBrushless); // Bag Motor

    chooserMotor.restoreFactoryDefaults();
    // intakeMotor.config
    chooserMotor.setSmartCurrentLimit(40);
    chooserMotor.setInverted(true);
    intakeOutput = new DebugEntry<Double>(0.0, "Intake Motor Ouput", this);
    chooserOutput = new DebugEntry<Double>(0.0, "Chooser Motor Output", this);
  }

  // TODO: Add check to make sure turret is below 45 degrees before running & add photogate when
  // implemented.
  public void runIntake() {
    intakeMotor.set(Constants.IntakeConstants.INTAKE_PERCENTAGE);
  }

  public void reverseIntake() {
    intakeMotor.set(-Constants.IntakeConstants.INTAKE_PERCENTAGE);
    chooserMotor.set(-IntakeConstants.CHOOSER_PERCENTAGE);
  }

  public void speakerChooser() {
    chooserMotor.set(Constants.IntakeConstants.CHOOSER_PERCENTAGE);
  }

  public void ampChooser() {
    chooserMotor.set(-Constants.IntakeConstants.CHOOSER_PERCENTAGE);
  }

  public void speakerIntake() {
    runIntake();
    speakerChooser();
  }

  public void ampIntake() {
    runIntake();
    ampChooser();
  }

  public void stopMotors() {
    chooserMotor.set(0);
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    intakeOutput.log(intakeMotor.get());
    chooserOutput.log(chooserMotor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

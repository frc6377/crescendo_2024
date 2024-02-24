// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.stateManagement.PlacementMode;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMotor;
  private CANSparkMax chooserMotor;
  private GenericEntry intakeOutput;
  private GenericEntry chooserOutput;
  private ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID, "Default Name");
    chooserMotor =
        new CANSparkMax(
            Constants.IntakeConstants.INTAKE_CHOOSER_ID, MotorType.kBrushed); // Bag Motor
    chooserMotor.restoreFactoryDefaults();
    // intakeMotor.config
    chooserMotor.setSmartCurrentLimit(20);
    intakeOutput = intakeTab.add("Intake Motor Output", 0).withPosition(3, 0).getEntry();
    chooserOutput = intakeTab.add("Chooser Motor Output", 0).withPosition(3, 1).getEntry();
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

  public Command reverseIntakeCommand() {
    return new StartEndCommand(this::reverseIntake, this::stopMotors, this)
        .withName("Reverse Intake");
  }

  // Runs the speaker intake or amp intake based on the robot state provided
  public Command getIntakeCommand(PlacementMode mode) {
    return buildIntakeCommand(mode.equals(PlacementMode.SPEAKER)).withName("getIntakeCommand");
  }

  public Command getSpeakerIntakeCommand() {
    return buildIntakeCommand(true).withName("getSpeakerIntakeCommnad");
  }

  public Command getAmpIntakeCommand() {
    return buildIntakeCommand(false).withName("getAmpIntakeCommand");
  }

  private Command buildIntakeCommand(boolean isSpeaker) {
    return new StartEndCommand(
            isSpeaker ? this::speakerIntake : this::ampIntake, this::stopMotors, this)
        .withName("Build Intake Command");
  }

  @Override
  public void periodic() {
    intakeOutput.setDouble(intakeMotor.get());
    chooserOutput.setDouble(chooserMotor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

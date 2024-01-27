// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.stateManagement.PlacementMode;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeMotor;
  private CANSparkMax chooserMotor;

  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    chooserMotor =
        new CANSparkMax(Constants.IntakeConstants.INTAKE_CHOOSER_ID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    chooserMotor.restoreFactoryDefaults();
    chooserMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(40);
    chooserMotor.setSmartCurrentLimit(20);
  }

  // TODO: Add check to make sure turret is below 45 degrees before running & add photogate when
  // implemented.
  public void runIntake() {
    intakeMotor.set(Constants.IntakeConstants.INTAKE_PERCENTAGE);
  }

  public void reverseIntake(){
    intakeMotor.set(-Constants.IntakeConstants.INTAKE_PERCENTAGE);
  }

  public void runChooser() {
    chooserMotor.set(Constants.IntakeConstants.CHOOSER_PERCENTAGE);
  }

  public void reverseChooser() {
    chooserMotor.set(-Constants.IntakeConstants.CHOOSER_PERCENTAGE);
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
  
  public Command reverseIntakeCommand() {
    return new StartEndCommand(this::reverseIntake, this::stopMotors, this);
  }
 
  // Runs the speaker intake or trap intake based on the robot state provided
  public Command getIntakeCommand(PlacementMode mode) {
    return buildIntakeCommand(mode.equals(PlacementMode.SPEAKER));
  }

  public Command getSpeakerIntakeCommand(){
    return buildIntakeCommand(true);
  }

  public Command getTrapIntakeCommand(){
    return buildIntakeCommand(false);
  }

  private Command buildIntakeCommand(boolean isSpeaker){
    return new StartEndCommand(
      isSpeaker ? this::speakerIntake : this::trapIntake,
      this::stopMotors,
      this);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

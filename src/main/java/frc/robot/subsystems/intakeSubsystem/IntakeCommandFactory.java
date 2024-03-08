package frc.robot.subsystems.intakeSubsystem;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.stateManagement.PlacementMode;

public class IntakeCommandFactory {
  private final IntakeSubsystem subsystem;

  public IntakeCommandFactory(IntakeSubsystem subsystem) {
    this.subsystem = subsystem;
    if (subsystem != null) Shuffleboard.getTab(subsystem.getName()).add(subsystem);
  }

  public Command reverseIntakeCommand() {
    if (subsystem == null) return Commands.none();
    return new StartEndCommand(subsystem::reverseIntake, subsystem::stopMotors, subsystem)
        .withName("reverseIntakeCommand")
        .asProxy();
  }

  // Runs the speaker intake or amp intake based on the robot state provided
  public Command getIntakeCommand(PlacementMode mode) {
    if (subsystem == null) return Commands.none();
    return buildIntakeCommand(mode.equals(PlacementMode.SPEAKER))
        .withName("getIntakeCommand")
        .asProxy();
  }

  public Command getSpeakerIntakeCommand() {
    if (subsystem == null) return Commands.none();
    return buildIntakeCommand(true).withName("getSpeakerIntakeCommand").asProxy();
  }

  public Command getAmpIntakeCommand() {
    if (subsystem == null) return Commands.none();
    return buildIntakeCommand(false).withName("getAmpIntakeCommand").asProxy();
  }

  private Command buildIntakeCommand(boolean isSpeaker) {
    if (subsystem == null) return Commands.none();
    return new StartEndCommand(
            isSpeaker ? subsystem::speakerIntake : subsystem::ampIntake,
            subsystem::stopMotors,
            subsystem)
        .withName("Build Intake Command");
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }
}

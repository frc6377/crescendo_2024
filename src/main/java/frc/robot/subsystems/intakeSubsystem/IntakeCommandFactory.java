package frc.robot.subsystems.intakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.stateManagement.PlacementMode;
import java.util.ArrayList;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class IntakeCommandFactory {
  private final IntakeSubsystem subsystem;

  public IntakeCommandFactory(IntakeSubsystem subsystem) {
    this.subsystem = subsystem;
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

  public Command intakeSourceForTime() {
    if (subsystem == null) return Commands.none();
    return Commands.deadline(
        new WaitCommand(ShooterConstants.INTAKE_DELAY_SEC), getSpeakerIntakeCommand());
  }

  public Command intakeSpeakerCommandSmart(BooleanSupplier tof) {
    if (subsystem == null) return Commands.none();
    return getSpeakerIntakeCommand().until(tof).andThen(intakeSourceForTime());
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
    subsystem.setDefaultCommand(Commands.defer(() -> defaultCommand, Set.of(subsystem)));
  }

  public Command[] getCommands() {
    ArrayList<Command> cmds = new ArrayList<Command>();
    cmds.add(reverseIntakeCommand());
    cmds.add(getIntakeCommand(PlacementMode.AMP));
    cmds.add(getSpeakerIntakeCommand());
    cmds.add(intakeSourceForTime());
    cmds.add(intakeSpeakerCommandSmart(() -> false));
    cmds.add(getAmpIntakeCommand());
    return cmds.toArray(new Command[cmds.size()]);
  }
}

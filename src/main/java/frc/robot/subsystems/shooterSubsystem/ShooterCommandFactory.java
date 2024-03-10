package frc.robot.subsystems.shooterSubsystem;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;

public class ShooterCommandFactory {
  private final ShooterSubsystem subsystem;
  private ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterSubsystem");
  private GenericEntry leftTargetRPM = shooterTab.add("Left motor target RPM", 2750).getEntry();
  private GenericEntry rightTargetRPM = shooterTab.add("Right motor target RPM", 2350).getEntry();

  public ShooterCommandFactory(ShooterSubsystem subsystem) {
    this.subsystem = subsystem;
  }

  public Command intakeSource() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .startEnd(
            () -> {
              subsystem.setShooterSpeeds(ShooterConstants.SHOOTER_SOURCE_INTAKE);
            },
            () -> {})
        .withName("intakeSource")
        .asProxy();
  }

  public Command intakeSourceForTime() {
    if (subsystem == null) return Commands.none();
    return Commands.deadline(new WaitCommand(ShooterConstants.INTAKE_DELAY_SEC), intakeSource())
        .withName("intakeSourceForTime")
        .asProxy();
  }

  public Command intakeSpeakerSource() {
    if (subsystem == null) return Commands.none();
    return intakeSource()
        .until(subsystem.getBeamBreak())
        .andThen(intakeSourceForTime())
        .withName("intakeSpeakerSource")
        .asProxy();
  }

  // Spins up the shooter, and requests feeding it when the rollers are within parameters.
  // Receives distance-to-target from Limelight, or other sensor.
  // Required to be called repeatedly; consider pub-sub for LimelightGetDistance() or equivalent
  // method to save a method call
  public Command revShooter() {
    if (subsystem == null) return Commands.none();
    return new FunctionalCommand(
        () -> {
          subsystem.setShooterSpeeds(ShooterSubsystem.calculateShooterSpeeds(170)); // Amp shoot
        },
        () -> {},
        (a) -> {},
        () -> false,
        subsystem);
  }

  // Idle shooter command
  public Command shooterIdle() {
    if (subsystem == null) return Commands.none();
    final Command command =
        subsystem
            .run(
                () -> {
                  subsystem.stopAndLogMotors();
                })
            .withName("Idle Shooter command");
    return command;
  }

  public Command outtake() {
    if (subsystem == null) return Commands.none();
    return subsystem
        .startEnd(() -> subsystem.requestPercent(-1), subsystem::stopAndLogMotors)
        .withName("Outtake command")
        .asProxy();
  }

  public void setDefaultCommand(Command defaultCommand) {
    if (subsystem == null) return;
    subsystem.setDefaultCommand(defaultCommand);
  }
}

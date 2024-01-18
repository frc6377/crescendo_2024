// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/** An example command that uses an example subsystem. */
public class TurretMagicCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TurretSubsystem turretSubsystem;
  private final Rotation2d limelightRotation;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurretMagicCommand(final TurretSubsystem turretSubsystem, Rotation2d limelightRotation) {
    this.turretSubsystem = turretSubsystem;
    this.limelightRotation = limelightRotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretSubsystem.magicMethod(limelightRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    turretSubsystem.setTurretVelo(0.0);
    turretSubsystem.setTurretPos(turretSubsystem.getTurretPos());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

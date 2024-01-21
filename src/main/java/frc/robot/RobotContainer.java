// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TrapArmSubsystem;
import frc.robot.subsystems.color.SignalingSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final SwerveSubsystem drivetrain;

  private final SignalingSubsystem signalingSubsystem =
      new SignalingSubsystem(1, OI.Driver::setRumble);

  private final DynamicRobotConfig dynamicRobotConfig;

  // private final RobotStateManager robotStateManager = new RobotStateManager();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    dynamicRobotConfig = new DynamicRobotConfig();
    drivetrain = dynamicRobotConfig.getTunerConstants().drivetrain;
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    OI.getTrigger(OI.Driver.intakeTrigger).whileTrue(intakeSubsystem.getIntakeCommand());
    OI.getButton(OI.Driver.outtakeButton).whileTrue(intakeSubsystem.getOuttakeCommand());
    // Swerve config
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drivetrain.getDriveRequest(
                    OI.getAxisSupplier(OI.Driver.xTranslationAxis).get(),
                    OI.getAxisSupplier(OI.Driver.yTranslationAxis).get(),
                    OI.getAxisSupplier(OI.Driver.rotationAxis).get())));
    OI.getButton(OI.Driver.brakeButton)
        .whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
    OI.getButton(OI.Driver.resetRotationButton)
        .onTrue(
            drivetrain.runOnce(
                () ->
                    drivetrain.seedFieldRelative(
                        new Pose2d(
                            drivetrain.getState().Pose.getTranslation(),
                            Rotation2d.fromDegrees(270)))));
    OI.getButton(OI.Driver.orientationButton)
        .onTrue(drivetrain.runOnce(() -> drivetrain.toggleOrientation()));
    // OI.Driver.getZeroButton().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().reset()));

    m_driverController.x().onTrue(trapArmSubsystem.intakeSource());
    m_driverController.x().onFalse(trapArmSubsystem.stop());
    m_driverController.y().onTrue(trapArmSubsystem.intakeGround());
    m_driverController.y().onFalse(trapArmSubsystem.stop());
    m_driverController.povUp().onTrue(trapArmSubsystem.scoreAMP());
    m_driverController.povUp().onFalse(trapArmSubsystem.stop());
    m_driverController.povDown().onTrue(trapArmSubsystem.scoreTrap());
    m_driverController.povDown().onFalse(trapArmSubsystem.stop());

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
  }

  public void onDisabled() {
    signalingSubsystem.randomizePattern();
  }

  public void onExitDisabled() {
    signalingSubsystem.clearLEDs();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto();
  }
}

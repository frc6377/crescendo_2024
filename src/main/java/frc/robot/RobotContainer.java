// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotStateManager.PlacementMode;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TrapElvSubsystem;
import frc.robot.subsystems.signaling.SignalingSubsystem;
import java.util.HashMap;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DynamicRobotConfig dynamicRobotConfig;

  private final SwerveSubsystem drivetrain;
  private final IntakeSubsystem intakeSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final SignalingSubsystem signalingSubsystem;

  private final TrapElvSubsystem trapElvSubsystem;

  private SendableChooser<Command> autoChooser;
  private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
  private GenericEntry autoDelay =
      configTab
          .add("Auton Start Delay(seconds)", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 2))
          .getEntry();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    dynamicRobotConfig = new DynamicRobotConfig();

    drivetrain = dynamicRobotConfig.getTunerConstants().drivetrain;
    intakeSubsystem = new IntakeSubsystem();
    limelightSubsystem = new LimelightSubsystem(drivetrain.getVisionMeasurementConsumer());
    signalingSubsystem = new SignalingSubsystem(1, OI.Driver::setRumble);

    trapElvSubsystem = new TrapElvSubsystem();

    configureBindings();
    registerCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    configTab.add("Auton Selection", autoChooser).withSize(3, 1);
    SmartDashboard.putBoolean("NamedCommand test", false);
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
    OI.getButton(OI.Driver.highGearButton).whileTrue(drivetrain.getHighGearCommand());
    OI.getButton(OI.Driver.resetRotationButton).onTrue(drivetrain.getResetRotationCommand());
    OI.getButton(OI.Driver.orientationButton)
        .onTrue(drivetrain.getToggleOrientationCommand())
        .whileTrue(
            signalingSubsystem
                .getContinuousStrobeCommand(Constants.STROBE_DELAY)
                .onlyIf(drivetrain::getIsFieldOriented));

    // Guidance
    OI.getPOVButton(OI.Driver.sourceGuidanceButton)
        .whileTrue(drivetrain.handleSourceGuidance())
        .onTrue(new InstantCommand(() -> RobotStateManager.setPlacementMode(PlacementMode.SOURCE)));
    OI.getPOVButton(OI.Driver.ampGuidanceLeftButton)
        .whileTrue(drivetrain.handleAmpGuidance())
        .onTrue(new InstantCommand(() -> RobotStateManager.setPlacementMode(PlacementMode.AMP)));
    OI.getPOVButton(OI.Driver.ampGuidanceRightButton)
        .whileTrue(drivetrain.handleAmpGuidance())
        .onTrue(new InstantCommand(() -> RobotStateManager.setPlacementMode(PlacementMode.AMP)));
    OI.getButton(OI.Driver.speakerRotateButton)
        .whileTrue(
            new RepeatCommand(new InstantCommand(() -> drivetrain.setSpeakerAutorotate()))
                .andThen(new InstantCommand(() -> drivetrain.endAutorotate())));

    OI.getPOVButton(OI.Operator.toggleSourceAutopilotButton)
        .onTrue(new InstantCommand(() -> drivetrain.toggleSourceAutopilot()));
    OI.getPOVButton(OI.Operator.toggleAmpAutopilotLeftButton)
        .onTrue(new InstantCommand(() -> drivetrain.toggleAmpAutopilot()));
    OI.getPOVButton(OI.Operator.toggleAmpAutopilotRightButton)
        .onTrue(new InstantCommand(() -> drivetrain.toggleAmpAutopilot()));

    // Trap Elv Intaking
    OI.getButton(OI.Driver.groundIntakeButton)
        .whileTrue(trapElvSubsystem.intakeGround().onlyWhile(trapElvSubsystem.getGroundBreak()));
    OI.getButton(OI.Driver.sourceIntakeButton)
        .whileTrue(trapElvSubsystem.intakeSource().onlyWhile(trapElvSubsystem.getSourceBreak()));

    // Trap Elv Scoring
    OI.getTrigger(OI.Driver.ampScoreTrigger).whileTrue(trapElvSubsystem.scoreAMP());
    OI.getButton(OI.Driver.trapScoreButton).whileTrue(trapElvSubsystem.scoreTrap());

    // Trap Elv zeroing button
    OI.getButton(OI.Driver.zeroArm).whileTrue(trapElvSubsystem.zeroArm());

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
  }

  // Register commands for auton
  public void registerCommands() {
    HashMap<String, Command> autonCommands = new HashMap<String, Command>();

    autonCommands.put("Shoot", autonTest());

    NamedCommands.registerCommands(autonCommands);
  }

  public void onDisabled() {
    signalingSubsystem.randomizePattern();
  }

  public void onExitDisabled() {
    signalingSubsystem.clearLEDs();
  }

  private Command autonTest() {
    return new InstantCommand(() -> SmartDashboard.putBoolean("NamedCommand test", true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous(including the delay)
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(autoDelay.getDouble(0)).andThen(autoChooser.getSelected());
  }
}

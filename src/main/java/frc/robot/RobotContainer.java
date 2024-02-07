// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TrapElvSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
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
  private static final double MaxSpeed = 6; // 6 meters per second desired top speed
  private static final double MaxAngularRate =
      Math.PI; // Half a rotation per second max angular velocity

  private final RobotStateManager robotStateManager = new RobotStateManager();

  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final TriggerSubsystem triggerSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final SwerveSubsystem drivetrain;
  private final LimelightSubsystem limelightSubsystem;

  private final SignalingSubsystem signalingSubsystem =
      new SignalingSubsystem(1, OI.Driver::setRumble, robotStateManager);

  private final TrapElvSubsystem trapElvSubsystem = new TrapElvSubsystem();

  private final DynamicRobotConfig dynamicRobotConfig;

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
    triggerSubsystem = new TriggerSubsystem();
    limelightSubsystem = new LimelightSubsystem(drivetrain.getVisionMeasurementConsumer());
    // Configure the trigger bindings
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
    OI.getButton(OI.Operator.switchPlacement)
        .onTrue(
            new InstantCommand(robotStateManager::switchPlacementMode)
                .withName("Switch Placement Mode Command"));
    OI.getTrigger(OI.Driver.intakeTrigger)
        .whileTrue(
            intakeSubsystem
                .getIntakeCommand(robotStateManager.getPlacementMode())
                .alongWith(trapElvSubsystem.intake(robotStateManager.getPlacementMode()))
                .withName("Get Placement Mode Command"));
    OI.getButton(OI.Driver.outtakeButton)
        .whileTrue(intakeSubsystem.reverseIntakeCommand().withName("Reverse Intake Command"));

    // Swerve config
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain
            .applyRequest(
                () ->
                    drivetrain.getDriveRequest(
                        OI.getAxisSupplier(OI.Driver.xTranslationAxis).get(),
                        OI.getAxisSupplier(OI.Driver.yTranslationAxis).get(),
                        OI.getAxisSupplier(OI.Driver.rotationAxis).get()))
            .withName("Get Axis Suppliers"));
    OI.getButton(OI.Driver.brakeButton)
        .whileTrue(
            drivetrain
                .applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
                .withName("Brake Swerve"));
    OI.getButton(OI.Driver.resetRotationButton)
        .onTrue(
            drivetrain
                .runOnce(
                    () ->
                        drivetrain.seedFieldRelative(
                            new Pose2d(
                                drivetrain.getState().Pose.getTranslation(),
                                Rotation2d.fromDegrees(180))))
                .withName("Put Pose & Rotation on Field"));
    OI.getButton(OI.Driver.orientationButton)
        .onTrue(
            drivetrain
                .runOnce(() -> drivetrain.toggleOrientation())
                .withName("Toggle Orientation"));
    // OI.Driver.getZeroButton().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().reset()));

    // Shooter commands
    shooterSubsystem.setDefaultCommand(shooterSubsystem.shooterIdle());
    OI.getTrigger(OI.Operator.shooterTrigger).onTrue(shooterSubsystem.shooterFire());

    // Trap Elv Scoring
    // OI.getButton(OI.Driver.ampScoreButton).whileTrue(trapElvSubsystem.scoreAMP());
    // OI.getButton(OI.Driver.trapScoreButton).whileTrue(trapElvSubsystem.scoreTrap());

    // Trap Elv zeroing button
    OI.getButton(OI.Driver.zeroArm).whileTrue(trapElvSubsystem.zeroElv());

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
  }

  // Register commands for auton
  public void registerCommands() {
    HashMap<String, Command> autonCommands = new HashMap<String, Command>();

    autonCommands.put("Shoot", autonTest().withName("Shoot"));
    autonCommands.put(
        "Speaker Intake", intakeSubsystem.getSpeakerIntakeCommand().withName("Speaker Intake"));
    autonCommands.put("Amp Intake", intakeSubsystem.getAmpIntakeCommand().withName("Amp Intake"));

    NamedCommands.registerCommands(autonCommands);
  }

  public void onDisabled() {
    signalingSubsystem.randomizePattern();
  }

  public void onExitDisabled() {
    signalingSubsystem.clearLEDs();
  }

  private Command autonTest() {
    return new InstantCommand(() -> SmartDashboard.putBoolean("NamedCommand test", true))
        .withName("Test NamedCommand");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous(including the delay)
   */
  public Command getAutonomousCommand() {
    return new WaitCommand(autoDelay.getDouble(0))
        .andThen(autoChooser.getSelected())
        .withName("Get Auto Command");
  }
}

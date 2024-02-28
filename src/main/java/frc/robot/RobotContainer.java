// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.enabledSubsystems;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.climberSubsystem.ClimberCommandFactory;
import frc.robot.subsystems.climberSubsystem.ClimberSubsystem;
import frc.robot.subsystems.intakeSubsystem.IntakeCommandFactory;
import frc.robot.subsystems.intakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.shooterSubsystem.ShooterCommandFactory;
import frc.robot.subsystems.shooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.signaling.SignalingSubsystem;
import frc.robot.subsystems.swerveSubsystem.SwerveCommandFactory;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem.DriveInput;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem.DriveRequest;
import frc.robot.subsystems.trapElvSubsystem.TrapElvCommandFactory;
import frc.robot.subsystems.trapElvSubsystem.TrapElvSubsystem;
import frc.robot.subsystems.triggerSubsystem.TriggerCommandFactory;
import frc.robot.subsystems.triggerSubsystem.TriggerSubsystem;
import frc.robot.subsystems.turretSubsystem.TurretComandFactory;
import frc.robot.subsystems.turretSubsystem.TurretSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.PhotonSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

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
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TriggerSubsystem triggerSubsystem;
  private final TurretSubsystem turretSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final SwerveSubsystem drivetrain;
  private final VisionSubsystem visionSubsystem;

  private final SignalingSubsystem signalingSubsystem;

  private final TrapElvSubsystem trapElvSubsystem;

  private final ClimberSubsystem climberSubsystem;

  private final DynamicRobotConfig dynamicRobotConfig;

  private SendableChooser<Command> autoChooser;
  private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
  private GenericEntry autoDelay =
      configTab
          .add("Auton Start Delay(seconds)", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(Map.of("min", 0, "max", 2))
          .getEntry();

  private final ShooterCommandFactory shooterCommandFactory;
  private final SwerveCommandFactory drivetrainCommandFactory;
  private final IntakeCommandFactory intakeCommandFactory;
  private final TriggerCommandFactory triggerCommandFactory;
  private final TrapElvCommandFactory trapElvCommandFactory;
  private final TurretComandFactory turretCommandFactory;
  private final ClimberCommandFactory climberCommandFactory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    dynamicRobotConfig = new DynamicRobotConfig();
    if (enabledSubsystems.shooterEnabled) {
      shooterSubsystem = new ShooterSubsystem();
    } else {
      shooterSubsystem = null;
    }
    shooterCommandFactory = new ShooterCommandFactory(shooterSubsystem);
    if (enabledSubsystems.signalEnabled) {
      signalingSubsystem = new SignalingSubsystem(1, OI.Driver::setRumble, robotStateManager);
    } else {
      signalingSubsystem = null;
    }
    if (enabledSubsystems.drivetrainEnabled) {
      drivetrain = dynamicRobotConfig.getTunerConstants().drivetrain;
    } else {
      drivetrain = null;
    }
    drivetrainCommandFactory = new SwerveCommandFactory(drivetrain);
    if (enabledSubsystems.intakeEnabled) {
      intakeSubsystem = new IntakeSubsystem();
    } else {
      intakeSubsystem = null;
    }
    intakeCommandFactory = new IntakeCommandFactory(intakeSubsystem);
    if (enabledSubsystems.triggerEnabled) {
      triggerSubsystem = new TriggerSubsystem();
    } else {
      triggerSubsystem = null;
    }
    triggerCommandFactory = new TriggerCommandFactory(triggerSubsystem);
    if (enabledSubsystems.visionEnabled) {
      visionSubsystem =
          Constants.enabledSubsystems.usingPhoton
              ? new PhotonSubsystem(drivetrain.getVisionMeasurementConsumer())
              : new LimelightSubsystem(
                  drivetrain.getVisionMeasurementConsumer(), robotStateManager);
    } else {
      visionSubsystem = new VisionSubsystem() {};
    }
    if (enabledSubsystems.elvEnabled) {
      trapElvSubsystem = new TrapElvSubsystem();
    } else {
      trapElvSubsystem = null;
    }
    trapElvCommandFactory = new TrapElvCommandFactory(trapElvSubsystem);
    if (enabledSubsystems.turretEnabled) {
      turretSubsystem = new TurretSubsystem(robotStateManager, null);
    } else {
      turretSubsystem = null;
    }
    turretCommandFactory = new TurretComandFactory(turretSubsystem);
    if (enabledSubsystems.climberEnabled) {
      climberSubsystem = new ClimberSubsystem();
    } else {
      climberSubsystem = null;
    }
    climberCommandFactory = new ClimberCommandFactory(climberSubsystem);

    if (Constants.enabledSubsystems.drivetrainEnabled) {
      registerCommands();
      autoChooser = AutoBuilder.buildAutoChooser();
      configTab.add("Auton Selection", autoChooser).withSize(3, 1);
      SmartDashboard.putBoolean("NamedCommand test", false);
    }

    if (Robot.isSimulation() && Constants.enabledSubsystems.drivetrainEnabled) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

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
    // Swerve config
    Supplier<DriveRequest> input =
        () ->
            SwerveSubsystem.joystickCondition(
                new DriveInput(
                    OI.getAxisSupplier(OI.Driver.xTranslationAxis).get(),
                    OI.getAxisSupplier(OI.Driver.yTranslationAxis).get(),
                    OI.getAxisSupplier(OI.Driver.rotationAxis).get()),
                0.1);

    drivetrainCommandFactory.setDefaultCommand(
        drivetrainCommandFactory.fieldOrientedDrive(input).withName("Get Axis Suppliers"));

    trapElvCommandFactory.setDefaultCommand(trapElvCommandFactory.stowTrapElvCommand());

    shooterCommandFactory.setDefaultCommand(shooterCommandFactory.shooterIdle());

    triggerCommandFactory.setDefaultCommand(triggerCommandFactory.getHoldCommand());

    OI.getButton(OI.Driver.resetRotationButton)
        .onTrue(drivetrainCommandFactory.zeroDriveTrain().withName("Put Pose & Rotation on Field"));

    OI.getButton(OI.Driver.useRod).whileTrue(drivetrainCommandFactory.robotOrientedDrive(input));

    OI.getTrigger(OI.Operator.prepareToFire)
        .whileTrue(
            Commands.either(
                    trapElvCommandFactory.positionAMP(),
                    prepareToScoreSpeaker(),
                    robotStateManager.isAmpSupplier())
                .andThen());

    OI.getTrigger(OI.Operator.fire)
        .whileTrue(
            Commands.either(
                trapElvCommandFactory.scoreAMP(),
                shootSpeaker(),
                robotStateManager.isAmpSupplier()));

    OI.getButton(OI.Operator.switchToAmp).onTrue(robotStateManager.setAmpMode());
    OI.getButton(OI.Operator.swtichToSpeaker).onTrue(robotStateManager.setSpeakerMode());

    OI.getTrigger(OI.Driver.intake)
        .whileTrue(
            Commands.either(intakeAmp(), intakeSpeaker(), robotStateManager.isAmpSupplier()));

    OI.getTrigger(OI.Driver.outtake).whileTrue(intakeCommandFactory.reverseIntakeCommand());

    OI.getButton(OI.Driver.intakeSource).whileTrue(trapElvCommandFactory.wristintakeSource());

    OI.getButton(OI.Driver.speakerSource).whileTrue(speakerSource());

    OI.getButton(OI.Operator.prepClimb).onTrue(climberCommandFactory.raise());

    OI.getButton(OI.Operator.latchClimber).onTrue(climberCommandFactory.clip());

    OI.getButton(OI.Operator.retractClimber).onTrue(climberCommandFactory.climb());
  }

  private Command speakerSource() {
    return Commands.deadline(
        shooterCommandFactory.intakeSource(), triggerCommandFactory.getLoadCommand());
  }

  private Command intakeSpeaker() {
    return Commands.parallel(
        intakeCommandFactory.getSpeakerIntakeCommand(), triggerCommandFactory.getLoadCommand());
  }

  private Command intakeAmp() {
    return Commands.parallel(
            trapElvCommandFactory.intakeGround(), intakeCommandFactory.getAmpIntakeCommand())
        .until(trapElvCommandFactory.getSourceBreak())
        .andThen(trapElvCommandFactory.intakeFromGroundForTime());
  }

  private Command shootSpeaker() {
    return Commands.either(
        triggerCommandFactory.getShootCommand(),
        triggerCommandFactory.getShootCommand().onlyIf(() -> shooterSubsystem.isShooterReady()),
        OI.getButton(OI.Operator.dumb));
  }

  private Command prepareToScoreSpeaker() {
    return Commands.parallel(
        turretCommandFactory.getAimTurretCommand(), shooterCommandFactory.revShooter());
  }

  // Register commands for auton
  public void registerCommands() {
    HashMap<String, Command> autonCommands = new HashMap<String, Command>();

    autonCommands.put("Shoot", autonTest().withName("Shoot"));
    if (Constants.enabledSubsystems.intakeEnabled) {
      autonCommands.put("Speaker Intake", intakeCommandFactory.getSpeakerIntakeCommand());
      autonCommands.put("Amp Intake", intakeCommandFactory.getAmpIntakeCommand());
    }
    autonCommands.put("Intake", new InstantCommand(() -> {}, new Subsystem[] {}));

    NamedCommands.registerCommands(autonCommands);
  }

  public void onDisabled() {
    if (Constants.enabledSubsystems.signalEnabled) {
      signalingSubsystem.randomizePattern();
    }
  }

  public void onExitDisabled() {
    if (Constants.enabledSubsystems.signalEnabled) {
      signalingSubsystem.clearLEDs();
    }
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
    if (Constants.enabledSubsystems.drivetrainEnabled) {
      return new WaitCommand(autoDelay.getDouble(0))
          .andThen(autoChooser.getSelected())
          .withName("Get Auto Command");
    }
    return null;
  }
}

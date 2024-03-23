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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.enabledSubsystems;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.stateManagement.AllianceColor;
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
import frc.robot.subsystems.turretSubsystem.TurretCommandFactory;
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
  final RobotStateManager robotStateManager = new RobotStateManager();

  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TriggerSubsystem triggerSubsystem;
  private final TurretSubsystem turretSubsystem;

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
  private final TurretCommandFactory turretCommandFactory;
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
    if (enabledSubsystems.turretRotationEnabled || enabledSubsystems.turretPitchEnabled) {
      turretSubsystem = new TurretSubsystem(robotStateManager, visionSubsystem);
    } else {
      turretSubsystem = null;
    }
    turretCommandFactory =
        new TurretCommandFactory(turretSubsystem, robotStateManager, drivetrain::getRotation);
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
    }

    if (Robot.isSimulation() && Constants.enabledSubsystems.drivetrainEnabled) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    configureBindings();
    configDriverFeedBack();
  }

  public void setVisionMeasuresEnabled(boolean enableVisionMeasures) {
    if (drivetrain != null) return;
    if (enableVisionMeasures) {
      drivetrain.startVisionMeasures();
    } else {
      drivetrain.stopVisionMeasures();
    }
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
                OI.getButton(OI.Driver.highGear).getAsBoolean());

    drivetrainCommandFactory.setDefaultCommand(
        drivetrainCommandFactory.fieldOrientedDrive(input).withName("Get Axis Suppliers"));

    trapElvCommandFactory.setDefaultCommand(trapElvCommandFactory.stowTrapElvCommand());

    shooterCommandFactory.setDefaultCommand(shooterCommandFactory.shooterIdle());

    triggerCommandFactory.setDefaultCommand(triggerCommandFactory.getHoldCommand());

    turretCommandFactory.setDefaultCommand(turretCommandFactory.idleTurret());

    OI.getButton(OI.Driver.resetRotationButton)
        .onTrue(drivetrainCommandFactory.zeroDriveTrain().withName("Put Pose & Rotation on Field"));

    OI.getButton(OI.Driver.useRod)
        .whileTrue(drivetrainCommandFactory.assistedDriving(input, robotStateManager));

    OI.getTrigger(OI.Operator.prepareToFire)
        .whileTrue(
            Commands.either(
                trapElvCommandFactory.positionAMP(),
                prepareToScoreSpeaker(),
                robotStateManager.isAmpSupplier()));

    OI.getTrigger(OI.Operator.fire)
        .whileTrue(
            Commands.either(
                trapElvCommandFactory.scoreAMP(),
                shootSpeaker(),
                robotStateManager.isAmpSupplier()));

    OI.getButton(OI.Operator.switchToAmp).onTrue(robotStateManager.setAmpMode());
    OI.getButton(OI.Operator.switchToSpeaker).onTrue(robotStateManager.setSpeakerMode());

    OI.getTrigger(OI.Driver.outtake).whileTrue(outtakeCommand());

    OI.getButton(OI.Driver.intakeSource).whileTrue(trapElvCommandFactory.wristintakeSource());

    OI.getButton(OI.Driver.speakerSource).whileTrue(speakerSource());

    OI.getButton(OI.Operator.prepClimb).onTrue(climberCommandFactory.raise());

    OI.getButton(OI.Operator.latchClimber).onTrue(climberCommandFactory.clip());

    OI.getButton(OI.Operator.retractClimber).toggleOnTrue(climberCommandFactory.climb());

    new Trigger(() -> OI.Operator.controller.getPOV() == 0).whileTrue(intakeCommand());
    new Trigger(() -> OI.Operator.controller.getPOV() == 180).whileTrue(outtakeCommand());
    new Trigger(() -> OI.Driver.controller.getPOV() == 0).whileTrue(intakeCommand());
  }

  private Command outtakeCommand() {
    return Commands.either(
        intakeCommandFactory.reverseIntakeCommand(),
        shooterOuttake(),
        robotStateManager.isAmpSupplier());
  }

  private Command intakeCommand() {
    return Commands.either(intakeAmp(), intakeSpeaker(), robotStateManager.isAmpSupplier());
  }

  private Command shooterOuttake() {
    return Commands.parallel(
            triggerCommandFactory.getEjectCommand(), intakeCommandFactory.reverseIntakeCommand())
        .asProxy();
  }

  private void configDriverFeedBack() {
    new Trigger(trapElvCommandFactory.getSourceBreak())
        .and(() -> OI.Operator.controller.getPOV() == 0)
        .whileTrue(
            Commands.startEnd(
                () -> {
                  OI.Driver.setRumble(Constants.OperatorConstants.RUMBLE_STRENGTH);
                  OI.Operator.setRumble(Constants.OperatorConstants.RUMBLE_STRENGTH);
                },
                () -> {
                  OI.Driver.setRumble(0);
                  OI.Operator.setRumble(0);
                }));
    new Trigger(shooterCommandFactory::isShooterReady)
        .whileTrue(
            Commands.startEnd(
                () -> OI.Operator.setRumble(Constants.OperatorConstants.RUMBLE_STRENGTH),
                () -> OI.Operator.setRumble(0)));
    shooterCommandFactory
        .getBeamBreak()
        .and(OI.getTrigger(OI.Driver.intake).or(() -> OI.Operator.controller.getPOV() == 00))
        .whileTrue(
            Commands.startEnd(
                () -> {
                  OI.Driver.setRumble(Constants.OperatorConstants.RUMBLE_STRENGTH);
                  OI.Operator.setRumble(Constants.OperatorConstants.RUMBLE_STRENGTH);
                },
                () -> {
                  OI.Driver.setRumble(0);
                  OI.Operator.setRumble(0);
                }));
  }

  private Command speakerSource() {
    return Commands.parallel(
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
        .andThen(trapElvCommandFactory.intakeFromGroundForTime())
        .asProxy();
  }

  private Command shootSpeaker() {
    return Commands.parallel(
        Commands.waitSeconds(CommandConstants.WAIT_FOR_TRAPELV)
            .andThen(
                Commands.either(
                    triggerCommandFactory.getShootCommand(),
                    triggerCommandFactory
                        .getShootCommand()
                        .onlyIf(() -> shooterCommandFactory.isShooterReady())
                        .asProxy(),
                    OI.getButton(OI.Operator.simple))),
        shooterCommandFactory.revShooter());
  }

  private Command prepareToScoreSpeaker() {
    return Commands.parallel(
        turretCommandFactory.getAimTurretCommand(),
        shooterCommandFactory.revShooter(),
        trapElvCommandFactory.wristShooterRev());
  }

  private Command prepareToScoreSpeakerShortRange() {
    return Commands.parallel(
        turretCommandFactory.shortRangeShot().asProxy(),
        shooterCommandFactory.revShooter(),
        trapElvCommandFactory.wristShooterRev());
  }

  private Command prepareToScoreSpeakerLongRange() {
    return Commands.parallel(
        turretCommandFactory.longRangeShot().asProxy(),
        shooterCommandFactory.revShooter(),
        trapElvCommandFactory.wristShooterRev());
  }

  private Command shootAutonShort() {
    return Commands.deadline(
        Commands.waitUntil(() -> shooterCommandFactory.isShooterReady())
            .andThen(
                triggerCommandFactory
                    .getShootCommand()
                    .withTimeout(1)
                    .until(shooterCommandFactory.getBeamBreak().negate().debounce(.25))),
        prepareToScoreSpeakerShortRange());
  }

  private Command shootAutonLong() {
    return Commands.deadline(
        Commands.waitUntil(() -> shooterCommandFactory.isShooterReady())
            .andThen(
                triggerCommandFactory
                    .getShootCommand()
                    .until(shooterCommandFactory.getBeamBreak().negate().debounce(.25))),
        prepareToScoreSpeakerLongRange());
  }

  private Command ampAuton() {
    return null; // return Commands.parallel(
    //     trapElvCommandFactory.positionAMP(),
    //     Commands.waitUntil(trapElvSubsystem.isAMPReady())
    //         .andThen(trapElvCommandFactory.scoreAMP()));
  }

  // Register commands for auton
  public void registerCommands() {
    HashMap<String, Command> autonCommands = new HashMap<String, Command>();

    autonCommands.put("Shoot", shootAutonShort());
    autonCommands.put("ShootLong", shootAutonLong());
    // autonCommands.put("Amp", ampAuton());
    if (Constants.enabledSubsystems.intakeEnabled) {
      autonCommands.put("Speaker Intake", intakeSpeaker().asProxy());
      autonCommands.put("Amp Intake", intakeAmp());
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

  public Translation2d feedSpeakerLocation() {
    if (robotStateManager.getAllianceColor() == AllianceColor.BLUE) {
      return Constants.FieldConstants.BLUE_SPEAKER;
    } else {
      return Constants.FieldConstants.RED_SPEAKER;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous(including the delay)
   */
  public Command getAutonomousCommand() {
    if (Constants.enabledSubsystems.drivetrainEnabled) {
      return new WaitCommand(autoDelay.getDouble(0))
          .andThen(autoChooser.getSelected().asProxy())
          .withName("Get Auto Command");
    }
    return null;
  }

  public void setOperatorPerspectiveForward(Rotation2d rotation) {
    if (drivetrain == null) return;
    drivetrain.setOperatorPerspectiveForward(rotation);
  }
}

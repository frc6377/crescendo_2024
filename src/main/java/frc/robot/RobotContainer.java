// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.enabledSubsystems;
import frc.robot.config.TunerConstants;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.PlacementMode;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.stateManagement.ShooterMode;
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
import frc.robot.utilities.TOFSensorSimple;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final RobotStateManager robotStateManager = new RobotStateManager();

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
    if (enabledSubsystems.shooterEnabled) {
      shooterSubsystem = new ShooterSubsystem();
    } else {
      shooterSubsystem = null;
    }
    shooterCommandFactory = new ShooterCommandFactory(shooterSubsystem, robotStateManager);
    if (enabledSubsystems.signalEnabled) {
      signalingSubsystem =
          new SignalingSubsystem(
              (a) -> {
                OI.Operator.setRumble(a);
                OI.Driver.setRumble(a);
              },
              robotStateManager);
    } else {
      signalingSubsystem = null;
    }
    if (enabledSubsystems.drivetrainEnabled) {
      drivetrain = TunerConstants.createDrivetrain(robotStateManager);
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
              ? new PhotonSubsystem(drivetrain.getVisionMeasurementConsumer(), robotStateManager)
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
      turretSubsystem = new TurretSubsystem(robotStateManager, null);
    } else {
      turretSubsystem = null;
    }
    turretCommandFactory =
        new TurretCommandFactory(
            turretSubsystem,
            robotStateManager,
            visionSubsystem,
            drivetrain::getRotation,
            drivetrainCommandFactory::currentRobotPosition);
    if (enabledSubsystems.climberEnabled) {
      climberSubsystem = new ClimberSubsystem();
    } else {
      climberSubsystem = null;
    }
    climberCommandFactory = new ClimberCommandFactory(climberSubsystem);

    if (Constants.enabledSubsystems.drivetrainEnabled) {
      registerCommands();
      autoChooser = AutoBuilder.buildAutoChooser("Lucy");
      configTab.add("Auton Selection", autoChooser).withSize(3, 1);
    }

    configureBindings();
    configDriverFeedBack();
  }

  public SwerveSubsystem getDriveTrain() {
    return drivetrain;
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
    final Supplier<DriveRequest> input =
        () ->
            SwerveSubsystem.joystickCondition(
                new DriveInput(
                    OI.getAxisSupplier(OI.Driver.xTranslationAxis).get(),
                    OI.getAxisSupplier(OI.Driver.yTranslationAxis).get(),
                    OI.getAxisSupplier(OI.Driver.rotationAxis).get()),
                OI.getButton(OI.Driver.highGear).getAsBoolean());
    final DoubleSupplier direction =
        drivetrainCommandFactory.createRotationSource(OI.Driver.controller, drivetrain);

    switch (DriverConstants.DRIVE_TYPE) {
      case FIELD_ORIENTED:
        drivetrainCommandFactory.setDefaultCommand(
            drivetrainCommandFactory.fieldOrientedDrive(input).withName("Field Oriented Drive"));
        break;
      case POINT_DRIVE:
        drivetrainCommandFactory.setDefaultCommand(
            drivetrainCommandFactory
                .pointDrive(direction, SwerveSubsystem.scrubRotation(input))
                .withName("Point Drive"));
        break;
      default:
        DriverStation.reportWarning("Unknown Drive Type Selected.", false);
        break;
    }
    trapElvCommandFactory.setDefaultCommand(trapElvCommandFactory.stowTrapElvCommand());

    shooterCommandFactory.setDefaultCommand(shooterCommandFactory.shooterIdle());

    triggerCommandFactory.setDefaultCommand(triggerCommandFactory.getHoldCommand());

    turretCommandFactory.setDefaultCommand(turretCommandFactory.idleTurret());

    OI.getTrigger(OI.Driver.lockAmp)
        .whileTrue(
            drivetrainCommandFactory.autoTargetAmp(
                SwerveSubsystem.scrubRotation(input), robotStateManager));
    OI.getButton(OI.Driver.lockSource)
        .whileTrue(
            drivetrainCommandFactory.autoTargetSource(
                SwerveSubsystem.scrubRotation(input), robotStateManager));
    OI.getTrigger(OI.Driver.lockSpeaker)
        .whileTrue(
            drivetrainCommandFactory.autoTargetSpeaker(
                SwerveSubsystem.scrubRotation(input), robotStateManager));

    OI.getButton(OI.Driver.resetRotationButton)
        .onTrue(drivetrainCommandFactory.zeroDriveTrain().withName("Put Pose & Rotation on Field"));

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

    OI.getButton(OI.Operator.prepClimb).onTrue(climberCommandFactory.raise());

    OI.getButton(OI.Operator.latchClimber).onTrue(climberCommandFactory.clip());

    OI.getButton(OI.Operator.retractClimber).toggleOnTrue(climberCommandFactory.climb());

    OI.getButton(OI.Operator.simple)
        .whileTrue(
            robotStateManager.setShooterMode(ShooterMode.SHORT_RANGE, ShooterMode.LONG_RANGE));

    new Trigger(() -> OI.Operator.controller.getPOV() == 0).whileTrue(intakeCommand());
    new Trigger(() -> OI.Operator.controller.getPOV() == 180).whileTrue(outtakeCommand());
    new Trigger(() -> OI.Operator.controller.getPOV() == 270)
        .whileTrue(robotStateManager.setShooterMode(ShooterMode.LOB, ShooterMode.LONG_RANGE));

    new Trigger(() -> OI.Driver.controller.getPOV() == 0).whileTrue(intakeCommand());
    OI.getButton(OI.Operator.disableOdomTracking)
        .whileTrue(robotStateManager.setShooterMode(ShooterMode.NO_ODOM, ShooterMode.LONG_RANGE));
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
        .and(() -> OI.Driver.controller.getPOV() == 0 || OI.Operator.controller.getPOV() == 0)
        .whileTrue(
            Commands.startEnd(
                () -> signalingSubsystem.startAmpSignal(), () -> signalingSubsystem.endSignal()));
    new Trigger(OI.getTrigger(OI.Operator.prepareToFire))
        .and(() -> robotStateManager.getPlacementMode() == PlacementMode.SPEAKER)
        .and(turretCommandFactory.isReadyTrigger())
        .and(shooterCommandFactory::isShooterReady)
        .whileTrue(
            Commands.startEnd(
                () -> signalingSubsystem.startShooterSignal(),
                () -> signalingSubsystem.endSignal()));
    shooterCommandFactory
        .getBeamBreak()
        .and(new Trigger(() -> OI.Operator.controller.getPOV() == 00))
        .whileTrue(
            Commands.startEnd(
                () -> signalingSubsystem.startIntakeSignal(),
                () -> signalingSubsystem.endSignal()));
    new TOFSensorSimple(3, 100)
        .beamBroken()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  OI.Driver.controller.setRumble(
                      RumbleType.kBothRumble, OperatorConstants.RUMBLE_STRENGTH);
                },
                () -> {
                  OI.Driver.controller.setRumble(RumbleType.kBothRumble, 0);
                },
                new Subsystem[0]));
  }

  private Command speakerSource() {
    return Commands.parallel(
        shooterCommandFactory.intakeSource(), triggerCommandFactory.getLoadCommand());
  }

  private Command intakeSpeaker() {
    return Commands.parallel(
        intakeCommandFactory.getSpeakerIntakeCommand().until(shooterCommandFactory.getBeamBreak()),
        triggerCommandFactory.getGroundLoadCommand(shooterCommandFactory.getBeamBreak()));
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
        triggerCommandFactory
            .getShootCommand()
            .onlyIf(() -> shooterCommandFactory.isShooterReady())
            .asProxy(),
        shooterCommandFactory.revShooter());
  }

  private Command prepareToScoreSpeaker() {
    return Commands.parallel(
        Commands.waitSeconds(CommandConstants.WAIT_FOR_TRAPELV)
            .andThen(turretCommandFactory.getAimTurretCommand()),
        shooterCommandFactory.revShooter(),
        trapElvCommandFactory.shooterMoving());
  }

  private Command prepareToScoreSpeakerShortRangeAutonOnly() {
    return Commands.parallel(
        turretCommandFactory.shortRangeShot().asProxy(),
        shooterCommandFactory.revShooter(),
        trapElvCommandFactory.shooterMoving());
  }

  private Command prepareToScoreSpeakerLongRangeAutonOnly() {
    return Commands.parallel(
        turretCommandFactory.longRangeShot().asProxy(),
        shooterCommandFactory.revShooter(),
        trapElvCommandFactory.shooterMoving());
  }

  private BooleanSupplier shooterAssemblyReady() {
    return turretCommandFactory
        .isReadyTrigger()
        .and(() -> shooterCommandFactory.isShooterReady())
        .debounce(.25);
  }

  private Command fire() {
    return triggerCommandFactory
        .getShootCommand()
        .until(shooterCommandFactory.getBeamBreak().negate().debounce(.25));
  }

  private Command fireWhenReady() {
    return Commands.waitUntil(shooterAssemblyReady()).andThen(fire());
  }

  private Command shootAutonShort() {
    return Commands.deadline(fireWhenReady(), prepareToScoreSpeakerShortRangeAutonOnly());
  }

  private Command shootAutonLong() {
    return Commands.deadline(
        Commands.waitUntil(
                turretCommandFactory
                    .isReadyTrigger()
                    .and(() -> shooterCommandFactory.isShooterReady())
                    .debounce(0.25))
            .andThen(
                triggerCommandFactory
                    .getShootCommand()
                    .until(shooterCommandFactory.getBeamBreak().negate().debounce(.25))),
        prepareToScoreSpeakerLongRangeAutonOnly());
  }

  private Command ampAuton() {
    return Commands.parallel(
        trapElvCommandFactory.positionAMP(),
        Commands.waitUntil(trapElvCommandFactory.getSourceBreak())
            .andThen(trapElvCommandFactory.scoreAMP())
            .onlyWhile(trapElvCommandFactory.getSourceBreak()));
  }

  // Register commands for auton
  public void registerCommands() {
    HashMap<String, Command> autonCommands = new HashMap<String, Command>();

    autonCommands.put("ShootShort", shootAutonShort());
    autonCommands.put("ShootLong", shootAutonLong());
    autonCommands.put("Prepare To Fire Short", prepareToScoreSpeakerShortRangeAutonOnly());
    autonCommands.put("Prepare To Fire Long", shooterCommandFactory.revShooter());
    autonCommands.put("Amp", ampAuton());
    if (Constants.enabledSubsystems.intakeEnabled) {
      autonCommands.put("Speaker Intake", intakeSpeaker().asProxy());
      autonCommands.put("Amp Intake", intakeAmp());
    }

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
}

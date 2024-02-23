// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SensorManager;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.DriveInput;
import frc.robot.subsystems.SwerveSubsystem.DriveRequest;
import frc.robot.subsystems.TrapElvSubsystem;
import frc.robot.subsystems.TriggerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.signaling.SignalingSubsystem;
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

  private final SwerveSubsystem drivetrain;
  private final VisionSubsystem visionSubsystem;

  private final SignalingSubsystem signalingSubsystem;

  private final TrapElvSubsystem trapElvSubsystem;

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
    if (Constants.enabledSubsystems.shooterEnabled) {
      shooterSubsystem = new ShooterSubsystem();
    } else {
      shooterSubsystem = null;
    }
    if (Constants.enabledSubsystems.signalEnabled) {
      signalingSubsystem = new SignalingSubsystem(1, OI.Driver::setRumble, robotStateManager);
    } else {
      signalingSubsystem = null;
    }
    dynamicRobotConfig = new DynamicRobotConfig();
    if (Constants.enabledSubsystems.drivetrainEnabled) {
      drivetrain = dynamicRobotConfig.getTunerConstants().drivetrain;
    } else {
      drivetrain = null;
    }
    if (Constants.enabledSubsystems.intakeEnabled) {
      intakeSubsystem = new IntakeSubsystem();
    } else {
      intakeSubsystem = null;
    }
    if (Constants.enabledSubsystems.triggerEnabled) {
      triggerSubsystem = new TriggerSubsystem();
    } else {
      triggerSubsystem = null;
    }
    if (Constants.enabledSubsystems.visionEnabled) {
      if (Constants.enabledSubsystems.drivetrainEnabled) {
        visionSubsystem =
            Constants.enabledSubsystems.usingPhoton
                ? new PhotonSubsystem(drivetrain.getVisionMeasurementConsumer())
                : new LimelightSubsystem(
                    drivetrain.getVisionMeasurementConsumer(), robotStateManager);
      } else {
        visionSubsystem = null;
      }
    } else {
      visionSubsystem = null;
    }
    if (Constants.enabledSubsystems.elvEnabled) {
      trapElvSubsystem = new TrapElvSubsystem();
    } else {
      trapElvSubsystem = null;
    }
    if (Constants.enabledSubsystems.turretEnabled) {
      if (Constants.enabledSubsystems.visionEnabled) {
        turretSubsystem = new TurretSubsystem(robotStateManager, visionSubsystem);
      } else {
        turretSubsystem = new TurretSubsystem(robotStateManager, null);
      }
      SmartDashboard.putData(turretSubsystem);
    } else {
      turretSubsystem = null;
    }
    // Configure the trigger bindings
    if (Constants.enabledSubsystems.drivetrainEnabled) {
      configureBindings();
      registerCommands();
      autoChooser = AutoBuilder.buildAutoChooser();
      configTab.add("Auton Selection", autoChooser).withSize(3, 1);
      SmartDashboard.putBoolean("NamedCommand test", false);
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
    OI.getButton(OI.Operator.A)
        .onTrue(
            new InstantCommand(robotStateManager::switchPlacementMode)
                .withName("Switch Placement Mode Command"));
    if (Constants.enabledSubsystems.intakeEnabled) {
      OI.getTrigger(OI.Driver.intakeTrigger)
          .whileTrue(
              intakeSubsystem
                  .getIntakeCommand(robotStateManager.getPlacementMode())
                  .withName("Get Placement Mode Command"));
      OI.getButton(OI.Driver.outtakeButton)
          .whileTrue(intakeSubsystem.reverseIntakeCommand().withName("Reverse Intake Command"));
    }
    // Swerve config
    if (Constants.enabledSubsystems.drivetrainEnabled) {
      Supplier<DriveRequest> input =
          () ->
              SwerveSubsystem.joystickCondition(
                  new DriveInput(
                      OI.getAxisSupplier(OI.Driver.xTranslationAxis).get(),
                      OI.getAxisSupplier(OI.Driver.yTranslationAxis).get(),
                      OI.getAxisSupplier(OI.Driver.rotationAxis).get()),
                  0.1);
      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          drivetrain.fieldOrientedDrive(input).withName("Get Axis Suppliers"));

      OI.getButton(OI.Driver.brakeButton)
          .whileTrue(
              drivetrain
                  .applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
                  .withName("Brake Swerve"));
      OI.getTrigger(OI.Driver.pointForward)
          .toggleOnTrue(drivetrain.pointAtLocation(new Translation2d(16.4846, 4.1), input));
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
    }

    // OI.Driver.getZeroButton().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().reset()));

    // Shooter commands
    if (Constants.enabledSubsystems.shooterEnabled && Constants.enabledSubsystems.triggerEnabled) {
      shooterSubsystem.setDefaultCommand(shooterSubsystem.shooterIdle());
      if (Constants.enabledSubsystems.visionEnabled) {
        OI.getTrigger(OI.Operator.shooterRevTrigger).whileTrue(shooterSubsystem.revShooter());

        OI.getTrigger(OI.Operator.shooterFireTrigger)
            .whileTrue(
                triggerSubsystem
                    .getShootCommand()
                    .onlyIf(shooterSubsystem.shooterReady())
                    .onlyWhile(OI.getTrigger(OI.Operator.shooterRevTrigger)));
        OI.getButton(OI.Operator.A).whileTrue(triggerSubsystem.getLoadCommand());
      } else {
        OI.getTrigger(OI.Operator.shooterFireTrigger).whileTrue(shooterSubsystem.bumperShoot());
      }
    }

    // Turret commands
    if (Constants.enabledSubsystems.turretEnabled) {
      turretSubsystem.setDefaultCommand(turretSubsystem.idleTurret());
      OI.getButton(OI.Operator.B).toggleOnTrue(turretSubsystem.getAimTurretCommand());
      OI.getButton(OI.Operator.Y).onTrue(turretSubsystem.moveUpwards());
      OI.getButton(OI.Operator.X).whileTrue(turretSubsystem.testTurretCommand(75));
    }

    // Trap Elv Intaking
    if (Constants.enabledSubsystems.elvEnabled) {
      OI.getButton(OI.Driver.groundIntakeButton)
          .whileTrue(
              trapElvSubsystem.intakeGround().onlyWhile(SensorManager.getGroundBreakBoolInverse()));
      OI.getButton(OI.Driver.sourceIntakeButton)
          .whileTrue(
              trapElvSubsystem.intakeSource().onlyWhile(SensorManager.getSourceBreakBoolInverse()));

      // Trap Elv Scoring

      OI.getButton(OI.Driver.ampScoreButton).whileTrue(trapElvSubsystem.scoreAMP());
      OI.getButton(OI.Driver.trapScoreButton).whileTrue(trapElvSubsystem.scoreTrap());
      // Trap Elv zeroing button

      OI.getButton(OI.Driver.zeroArm).whileTrue(trapElvSubsystem.zeroArm());
    }

    if (Robot.isSimulation() && Constants.enabledSubsystems.drivetrainEnabled) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
  }

  // Register commands for auton
  public void registerCommands() {
    HashMap<String, Command> autonCommands = new HashMap<String, Command>();

    autonCommands.put("Shoot", autonTest().withName("Shoot"));
    if (Constants.enabledSubsystems.intakeEnabled) {
      autonCommands.put("Speaker Intake", intakeSubsystem.getSpeakerIntakeCommand());
      autonCommands.put("Amp Intake", intakeSubsystem.getAmpIntakeCommand());
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

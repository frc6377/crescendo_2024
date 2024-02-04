package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.OI;
import frc.robot.RobotStateManager;
import frc.robot.Telemetry;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
  private final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private final double maxSpeed =
      Units.feetToMeters(18.2); // Max angular linear velocity in radians per second
  private final double maxAngularSpeed = Math.PI; // Max angular velocity in radians per second
  private final SwerveDriveKinematics kinematics;
  private final Telemetry telemetry = new Telemetry(maxSpeed);
  private final double drivetrainRadius;

  private final double lowGearCoefficient = 0.7;

  // State handling
  private boolean isFieldOriented = true;
  private boolean isHighGear = false;
  private boolean isPointDrive = false;
  private boolean isSourceAutopilotEnabled = true;
  private boolean isAmpAutopilotEnabled = true;
  private Rotation2d autorotateSetpoint = null;

  public SwerveSubsystem(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

    drivetrainRadius =
        modules[0].LocationX
            * Math.sqrt(2); // 45-45-90 triangle - hypotenuse is side length * root(2)

    Translation2d[] kinematicsTranslations = new Translation2d[4];
    for (int i = 0; i < 4; i++) {
      kinematicsTranslations[i] = new Translation2d(modules[i].LocationX, modules[i].LocationY);
    }
    kinematics = new SwerveDriveKinematics(kinematicsTranslations);

    AutoBuilder.configureHolonomic(
        () -> super.getState().Pose,
        (a) -> super.seedFieldRelative(a),
        () -> getChassisSpeeds(),
        (a) ->
            this.setControl(
                new SwerveRequest.RobotCentric()
                    .withVelocityX(a.vxMetersPerSecond)
                    .withVelocityY(a.vyMetersPerSecond)
                    .withRotationalRate(a.omegaRadiansPerSecond)),
        new HolonomicPathFollowerConfig(
            maxSpeed * 0.85, drivetrainRadius, new ReplanningConfig(true, true)),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    this.registerTelemetry(telemetry::telemeterize);
  }

  public SwerveSubsystem(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    this(driveTrainConstants, 0, modules);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(super.getState().ModuleStates);
  }

  // xSpeed, ySpeed, rotationSpeed should be axes with range -1<0<1
  public SwerveRequest getDriveRequest(double xSpeed, double ySpeed, double rotationSpeed) {
    // Idle if receiving only zero-value inputs & no autorotate
    if (xSpeed == 0 && ySpeed == 0 && rotationSpeed == 0 && autorotateSetpoint == null) {
      return new SwerveRequest.Idle();
    }
    // Disable autorotate if rotationSpeed is greater than zero
    if (rotationSpeed > 0) {
      endAutorotate();
    }
    // Convert inputs to vector
    double magnitude = OI.Driver.translationMagnitudeCurve.calculate(Math.hypot(xSpeed, ySpeed));
    Rotation2d rotation = new Rotation2d(xSpeed, ySpeed);
    // Scale vector based on gearing
    if (!isHighGear) {
      magnitude *= lowGearCoefficient;
    }
    // Break vector into components
    xSpeed = magnitude * Math.cos(rotation.getRadians());
    ySpeed = magnitude * Math.sin(rotation.getRadians());
    // Scale components based on maximum speed
    xSpeed *= maxSpeed;
    ySpeed *= maxSpeed;
    rotationSpeed *= maxAngularSpeed;
    // All following y-x mixups are intentional, WPI has its coordinate plane from the perspective
    // of the scoring table
    // If not field oriented return ROD
    if (!isFieldOriented) {
      return new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withVelocityX(ySpeed)
          .withVelocityY(xSpeed)
          .withRotationalRate(rotationSpeed);
    }
    // If point drive return FOD with point drive
    if (isPointDrive) {
      return new SwerveRequest.FieldCentricFacingAngle()
          .withTargetDirection(rotation)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withVelocityX(ySpeed)
          .withVelocityY(xSpeed);
    }
    // If no autorotate setpoint return FOD
    if (autorotateSetpoint == null) {
      return new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withVelocityX(ySpeed)
          .withVelocityY(xSpeed)
          .withRotationalRate(rotationSpeed);
    }
    // Return FOD with autorotate
    SwerveRequest.FieldCentricFacingAngle autorotateRequest =
        new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(autorotateSetpoint)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withVelocityX(ySpeed)
            .withVelocityY(xSpeed);
    autorotateRequest.HeadingController.setPID(0.8, 0.0025, 0);
    return autorotateRequest;
  }

  public Command getResetRotationCommand() {
    return this.runOnce(
        () ->
            this.seedFieldRelative(
                new Pose2d(this.getState().Pose.getTranslation(), Rotation2d.fromDegrees(180))));
  }

  public Command getToggleOrientationCommand() {
    return this.runOnce(() -> this.toggleOrientation());
  }

  public void toggleOrientation() {
    isFieldOriented = !isFieldOriented;
    if (!isFieldOriented) {
      // robot oriented drive means we can't hold a field oriented heading
      endAutorotate();
    }
  }

  public boolean getIsFieldOriented() {
    return isFieldOriented;
  }

  public void setHighGear(boolean isHighGear) {
    this.isHighGear = isHighGear;
  }

  public Command getHighGearCommand() {
    return new StartEndCommand(() -> setHighGear(true), () -> setHighGear(false));
  }

  public void setPointDrive(boolean isPointDrive) {
    this.isPointDrive = isPointDrive;
  }

  public Command getPointDriveCommand() {
    return new StartEndCommand(() -> setPointDrive(true), () -> setPointDrive(false));
  }

  private Command getAutopilotCommand(Pose2d pose) {
    return AutoBuilder.pathfindToPose(
        pose,
        new PathConstraints(
            PathfindingConstants.maximumVelocity,
            PathfindingConstants.maximumAcceleration,
            PathfindingConstants.maximumAngularVelocity,
            PathfindingConstants.maximumAngularAccelaration));
  }

  private Pose2d getAutopilotSourcePose() {
    return RobotStateManager.getIsAllianceRed()
        ? PathfindingConstants.redSourcePose
        : PathfindingConstants.blueSourcePose;
  }

  private Pose2d getAutopilotAmpPose() {
    return RobotStateManager.getIsAllianceRed()
        ? PathfindingConstants.redAmpPose
        : PathfindingConstants.blueAmpPose;
  }

  private Command startGuidance(boolean autopilot, Pose2d pose) {
    setAutorotate(pose.getRotation());
    return autopilot ? getAutopilotCommand(pose) : new InstantCommand();
  }

  public Command handleSourceGuidance() {
    return startGuidance(isSourceAutopilotEnabled, getAutopilotSourcePose());
  }

  public Command handleAmpGuidance() {
    return startGuidance(isAmpAutopilotEnabled, getAutopilotAmpPose());
  }

  public void toggleSourceAutopilot() {
    isSourceAutopilotEnabled = !isSourceAutopilotEnabled;
  }

  public void toggleAmpAutopilot() {
    isAmpAutopilotEnabled = !isAmpAutopilotEnabled;
  }

  public void setAutorotate(Rotation2d setpoint) {
    autorotateSetpoint = setpoint;
  }

  public void setSpeakerAutorotate() {
    Pose2d speakerPose =
        RobotStateManager.getIsAllianceRed()
            ? PathfindingConstants.redAmpPose
            : PathfindingConstants.blueAmpPose;
    double x = Math.abs(speakerPose.getX() - getState().Pose.getX());
    double y = Math.abs(speakerPose.getY() - getState().Pose.getY());
    autorotateSetpoint = new Rotation2d(Math.atan2(y, x));
  }

  public void endAutorotate() {
    autorotateSetpoint = null;
  }

  public BiConsumer<Pose2d, Double> getVisionMeasurementConsumer() {
    return (t, u) -> addVisionMeasurement(t, u);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void periodic() {
    if (autorotateSetpoint != null) {
      SmartDashboard.putNumber("autorotate", autorotateSetpoint.getDegrees());
    }
    SmartDashboard.putBoolean("amp auto enabled", isAmpAutopilotEnabled);
    SmartDashboard.putBoolean("source auto enabled", isSourceAutopilotEnabled);
  }
}

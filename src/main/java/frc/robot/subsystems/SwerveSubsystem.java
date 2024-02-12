package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.Telemetry;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private static final double maxSpeed = Units.feetToMeters(18.2); // Desired top speed
  private static final double maxAngularRate =
      Math.PI; // Max angular velocity in radians per second
  private final double drivetrainRadius;
  private final Telemetry telemetry = new Telemetry(maxSpeed);

  private static boolean isFieldOriented = true;
  private static Rotation2d alignmentRotation = null;

  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private SwerveDriveKinematics kinematics;

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

    if (Robot.isSimulation()) {
      this.registerTelemetry(telemetry::telemeterize);
    } else {
      this.registerTelemetry(telemetry::realTelemetry);
    }
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
    double magnitude = OI.Driver.translationMagnitudeCurve.calculate(Math.hypot(xSpeed, ySpeed));
    Rotation2d rotation = new Rotation2d(xSpeed, ySpeed);
    xSpeed = magnitude * Math.cos(rotation.getRadians());
    ySpeed = magnitude * Math.sin(rotation.getRadians());
    if (xSpeed == 0 && ySpeed == 0 && rotationSpeed == 0) {
      return new SwerveRequest.Idle();
    }
    xSpeed *= maxSpeed;
    ySpeed *= maxSpeed;
    rotationSpeed *= maxAngularRate;
    if (isFieldOriented) {
      if (alignmentRotation != null) {
        return new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(alignmentRotation)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            // Mixup is intentional, WPI has its coordinate plane from the perspective of the
            // scoring table
            .withVelocityX(ySpeed)
            .withVelocityY(xSpeed);
      }
      return new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          // Mixup is intentional, WPI has its coordinate plane from the perspective of the scoring
          // table
          .withVelocityX(ySpeed)
          .withVelocityY(xSpeed)
          .withRotationalRate(rotationSpeed);
    }
    return new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        // Mixup is intentional, WPI has its coordinate plane from the perspective of the scoring
        // table
        .withVelocityX(ySpeed)
        .withVelocityY(xSpeed)
        .withRotationalRate(rotationSpeed);
  }

  public BiConsumer<Pose2d, Double> getVisionMeasurementConsumer() {
    return (t, u) -> addVisionMeasurement(t, u);
  }

  public SwerveRequest getBrakeRequest() {
    return new SwerveRequest.SwerveDriveBrake();
  }

  public SwerveRequest getAlignRequest(Rotation2d rotation) {
    return new SwerveRequest.FieldCentricFacingAngle().withTargetDirection(rotation);
  }

  public void toggleOrientation() {
    isFieldOriented = !isFieldOriented;
    if (!isFieldOriented) {
      // robot oriented drive means we can't hold a field oriented heading
      endAlignment();
    }
  }

  public void setAlignment(Rotation2d rotation) {
    alignmentRotation = rotation;
  }

  public void endAlignment() {
    alignmentRotation = null;
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get())).withName("Request Supplier");
  }

  /**
   * Request the robot to point in a specified direction
   *
   * @param angleToPoint the angle to point in degrees
   * @param xDemand the x demand 0-1
   * @param yDemand the y demant 0-1
   * @return A command that will point in the specified direction until interupted
   */
  public Command pointInDirection(
      Rotation2d angleToPoint, DoubleSupplier xDemand, DoubleSupplier yDemand) {
    return pointDrive(() -> angleToPoint.getDegrees(), xDemand, yDemand)
        .withName("Pointing in direction");
  }

  /**
   * Points a location on the field
   *
   * @param target the location to point at
   * @param xDemand the x demand 0-1
   * @param yDemand the y demant 0-1
   * @return A command that will point at the specified location until interupted
   */
  public Command pointAtLocation(
      Translation2d target, DoubleSupplier xDemand, DoubleSupplier yDemand) {
    DoubleSupplier getAngleToTarget =
        () -> {
          Translation2d delta = this.getState().Pose.getTranslation().minus(target);
          return new Rotation2d(delta.getX(), delta.getY()).getDegrees();
        };
    return pointDrive(getAngleToTarget, xDemand, yDemand).withName("Pointing at location");
  }

  /**
   * Makes the robot point in a specfic direction. While still being able to move in the other 2
   * axis
   *
   * @param targetAngleProvider - the target angle provider to point from robot 0 in degreees
   * @param xDemand the x demand 0-1
   * @param yDemand the y demant 0-1
   * @return A command that will point in the requested direction until interupted
   */
  public Command pointDrive(
      DoubleSupplier targetAngleProvider, DoubleSupplier xDemand, DoubleSupplier yDemand) {
    ProfiledPIDController pid =
        new ProfiledPIDController(
            Constants.SwerveDriveConstants.TURN_kP,
            0,
            Constants.SwerveDriveConstants.TURN_kD,
            new TrapezoidProfile.Constraints(
                Constants.SwerveDriveConstants.MAX_AUTO_TURN,
                Constants.SwerveDriveConstants.MAX_AUTO_ACCERLATION));
    Runnable command =
        () -> {
          pid.setGoal(targetAngleProvider.getAsDouble());
          double alpha = pid.calculate(getRotation3d().toRotation2d().getDegrees());
          this.setControl(
              new SwerveRequest.FieldCentric()
                  .withRotationalRate(alpha)
                  .withVelocityX(xDemand.getAsDouble())
                  .withVelocityY(yDemand.getAsDouble()));
        };

    return run(command).withName("Point Drive");
  }
}

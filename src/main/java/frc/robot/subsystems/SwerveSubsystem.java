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
import edu.wpi.first.math.MathUtil;
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
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get())).withName("Request Supplier");
  }

  /**
   * Request the robot to point in a specified direction
   *
   * @param angleToPoint the angle to point in degrees
   * @param input the X-Y speeds
   * @return A command that will point in the specified direction until interupted
   */
  public Command pointInDirection(Rotation2d angleToPoint, Supplier<DriveRequest> input) {
    return pointDrive(() -> angleToPoint.getDegrees(), input).withName("Pointing in direction");
  }

  /**
   * Points a location on the field
   *
   * @param target the location to point at
   * @param input X-Y speeds
   * @return A command that will point at the specified location until interupted
   */
  public Command pointAtLocation(Translation2d target, Supplier<DriveRequest> input) {
    DoubleSupplier getAngleToTarget =
        () -> {
          Translation2d delta = this.getState().Pose.getTranslation().minus(target);
          return new Rotation2d(delta.getX(), delta.getY()).getDegrees();
        };
    return pointDrive(getAngleToTarget, input).withName("Pointing at location");
  }

  /**
   * Makes the robot point in a specfic direction. While still being able to move in the other 2
   * axis
   *
   * @param targetAngleProvider - the target angle provider to point from robot 0 in degreees
   * @param input the human X-Y request
   * @return A command that will point in the requested direction until interupted
   */
  public Command pointDrive(DoubleSupplier targetAngleProvider, Supplier<DriveRequest> input) {
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
          DriveRequest in = input.get();
          pid.setGoal(targetAngleProvider.getAsDouble());
          double alpha = pid.calculate(getRotation3d().toRotation2d().getDegrees());
          this.setControl(
              new SwerveRequest.FieldCentric()
                  .withRotationalRate(alpha)
                  .withVelocityX(in.xSpeed() * maxSpeed)
                  .withVelocityY(in.ySpeed() * maxSpeed));
          System.out.println("I'm trying!");
        };

    return run(command).withName("Point Drive");
  }

  public Command robotOrientedDrive(Supplier<DriveRequest> requestSupplier) {
    Runnable command =
        () -> {
          DriveRequest driveRequest = requestSupplier.get();
          SwerveRequest swerveRequest =
              new SwerveRequest.RobotCentric()
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                  .withVelocityX(driveRequest.xSpeed() * maxSpeed)
                  .withVelocityY(driveRequest.ySpeed() * maxSpeed)
                  .withRotationalRate(driveRequest.alpha() * maxAngularRate);
          setControl(swerveRequest);
        };

    return run(command);
  }

  public Command fieldOrientedDrive(Supplier<DriveRequest> requestSupplier) {
    Runnable command =
        () -> {
          DriveRequest driveRequest = requestSupplier.get();
          SwerveRequest swerveRequest =
              new SwerveRequest.FieldCentric()
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                  .withVelocityX(driveRequest.xSpeed() * maxSpeed)
                  .withVelocityY(driveRequest.ySpeed() * maxSpeed)
                  .withRotationalRate(driveRequest.alpha() * maxAngularRate);
          setControl(swerveRequest);
        };

    return run(command);
  }

  public static DriveRequest joystickCondition(DriveInput input, double deadband) {
    double mag = Math.sqrt(input.x() * input.x() + input.y() * input.y());
    double finalAlpha = MathUtil.applyDeadband(input.alpha(), deadband);
    if (mag < deadband * deadband) {
      return new DriveRequest(0, 0, finalAlpha);
    }

    double finalMag = MathUtil.applyDeadband(mag, deadband);
    // Mixup is intentional, WPI has its coordinate plane from the perspective of the
    // scoring
    // table
    return new DriveRequest(input.y() * finalMag / mag, input.x() * finalMag / mag, finalAlpha);
  }

  // Similar to a struct from other languages
  // see ref for full explanation
  // Ref:
  // https://docs.oracle.com/en/java/javase/17/language/records.html#GUID-6699E26F-4A9B-4393-A08B-1E47D4B2D263
  public record DriveRequest(double xSpeed, double ySpeed, double alpha) {
    public double getMagnitude() {
      return Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);
    }
  }

  public record DriveInput(double x, double y, double alpha) {}
}

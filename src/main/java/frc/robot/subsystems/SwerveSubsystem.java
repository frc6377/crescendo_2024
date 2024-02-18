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
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
            maxSpeed * 0.1, drivetrainRadius, new ReplanningConfig(true, true)),
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
   * Request the robot to point in a specified direction. Any non zero rotation demand will result
   * in the command canceling.
   *
   * @param angleToPoint the angle to point in degrees
   * @param input the X-Y speeds
   * @return A command that will point in the specified direction until interupted
   */
  public Command pointInDirection(Rotation2d angleToPoint, Supplier<DriveRequest> input) {
    return pointDrive(() -> angleToPoint.getDegrees(), input).withName("Pointing in direction");
  }

  /**
   * Points toward a location on the field any non zero rotation demand will result in the command
   * canceling.
   *
   * @param target the location to point at in meters
   * @param input X-Y speeds
   * @return A command that will point at the specified location until interupted
   */
  public Command pointAtLocation(final Translation2d target, final Supplier<DriveRequest> input) {
    final DoubleSupplier getAngleToTarget =
        () -> {
          Translation2d delta = this.getState().Pose.getTranslation().minus(target);
          return new Rotation2d(delta.getX(), delta.getY()).getDegrees();
        };
    return pointDrive(getAngleToTarget, input).withName("Pointing at location");
  }

  /**
   * Makes the robot point in a specfic direction. While still being able to move in the other 2
   * axis any non zero rotation demand will result in the command canceling.
   *
   * @param targetAngleProvider - the target angle provider to point from robot 0 in degreees
   * @param input the human X-Y request
   * @return A command that will point in the requested direction until interupted
   */
  public Command pointDrive(
      final DoubleSupplier targetAngleProvider, final Supplier<DriveRequest> input) {
    final ProfiledPIDController pid =
        new ProfiledPIDController(
            Constants.SwerveDriveConstants.TURN_kP,
            0,
            Constants.SwerveDriveConstants.TURN_kD,
            new TrapezoidProfile.Constraints(
                Constants.SwerveDriveConstants.MAX_AUTO_TURN,
                Constants.SwerveDriveConstants.MAX_AUTO_ACCERLATION));
    pid.enableContinuousInput(0, 360);
    final Runnable init =
        () -> {
          pid.reset(getState().Pose.getRotation().getDegrees());
        };

    final Runnable exec =
        () -> {
          DriveRequest in = input.get();
          pid.setGoal(targetAngleProvider.getAsDouble());
          double alpha = Math.toRadians(pid.calculate(getState().Pose.getRotation().getDegrees()));
          SmartDashboard.putNumber("error", pid.getPositionError());
          this.setControl(
              new SwerveRequest.FieldCentric()
                  .withRotationalRate(alpha)
                  .withVelocityX(in.xSpeed() * maxSpeed)
                  .withVelocityY(in.ySpeed() * maxSpeed));
          if (in.alpha != 0) {
            this.getCurrentCommand().cancel();
          }
        };

    final Command command = new FunctionalCommand(init, exec, (interupt) -> {}, () -> false, this);
    return command.withName("Point Drive");
  }

  /**
   * Rotate to rotation with error from the target rotation being fed in.
   *
   * @param err The error from the target target rotation
   * @param input X-Y speed demands, non-zero alpha will cancel this demand
   * @return A command that rotates to attempts to zero the error given.
   */
  public Command rotationDrive(final Supplier<Rotation2d> err, final Supplier<DriveRequest> input) {
    final PIDController pid =
        new PIDController(
            Constants.SwerveDriveConstants.TURN_kP, 0, Constants.SwerveDriveConstants.TURN_kD);
    pid.enableContinuousInput(0, 360);
    pid.setSetpoint(0);
    final Runnable command =
        () -> {
          DriveRequest in = input.get();
          double alpha = Math.toRadians(pid.calculate(err.get().getDegrees()));
          this.setControl(
              new SwerveRequest.FieldCentric()
                  .withRotationalRate(alpha)
                  .withVelocityX(in.xSpeed() * maxSpeed)
                  .withVelocityY(in.ySpeed() * maxSpeed));
          if (in.alpha != 0) {
            this.getCurrentCommand().cancel();
          }
        };

    return run(command).finallyDo(() -> pid.close()).withName("Rotation Drive");
  }

  public Command robotOrientedDrive(final Supplier<DriveRequest> requestSupplier) {
    final Runnable command =
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

  public Command fieldOrientedDrive(final Supplier<DriveRequest> requestSupplier) {
    final Runnable command =
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

  public static DriveRequest joystickCondition(final DriveInput input, final double deadband) {
    final double mag = Math.sqrt(input.x() * input.x() + input.y() * input.y());
    final double finalAlpha =
        -OI.Driver.rotationCurve.calculate(MathUtil.applyDeadband(input.alpha(), deadband));
    if (mag < deadband * deadband) {
      return new DriveRequest(0, 0, finalAlpha);
    }

    final double finalMag =
        OI.Driver.translationMagnitudeCurve.calculate(MathUtil.applyDeadband(mag, deadband));
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

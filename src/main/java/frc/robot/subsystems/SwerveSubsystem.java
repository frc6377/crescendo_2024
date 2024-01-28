package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.OI;
import frc.robot.Telemetry;
import frc.robot.config.DynamicRobotConfig;
import java.util.function.BiConsumer;
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
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * A command to zero all the pods
   *
   * <p>Assumes that all the pods are zeroed before the command is ran
   *
   * @return A command to zero all the pods
   */
  public Command zeroPods() {
    return Commands.runOnce(
        () -> {
          MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
          magnetConfig.withMagnetOffset(0);
          CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
          canCoderConfig.withMagnetSensor(magnetConfig);

          CANcoder[] canCoders = new CANcoder[4];
          for (int i = 0; i < 4; i++) {
            canCoders[i] = Modules[i].getCANcoder();
            canCoders[i].getConfigurator().apply(canCoderConfig);
          }

          for (int i = 0; i < 4; i++) {
            Translation2d position = m_moduleLocations[i];

            //  -x +x
            // +y 2 3
            // -y 0 1
            int quadrent =
                (int)
                    ((Math.copySign(1, position.getX()) - 1) / 2
                        + -(Math.copySign(1, position.getY()) - 1));

            switch (quadrent) {
              case 0:
                DynamicRobotConfig.ConfigVariables.backLeftOffset =
                    getCancoderAbsolutePosition(canCoders[i]);
                break;
              case 1:
                DynamicRobotConfig.ConfigVariables.backRightOffset =
                    getCancoderAbsolutePosition(canCoders[i]);
                break;
              case 2:
                DynamicRobotConfig.ConfigVariables.frontLeftOffset =
                    getCancoderAbsolutePosition(canCoders[i]);
                break;
              case 3:
                DynamicRobotConfig.ConfigVariables.frontRightOffset =
                    getCancoderAbsolutePosition(canCoders[i]);
                break;
              default:
                break;
            }
          }
        },
        this);
  }

  /**
   * Get the cancoders current absolute position.
   *
   * <p>this will block until the next can cycle to garuntee any command sent would of taken effect.
   *
   * @param coder The CANcoder to get the position
   * @return the current position in revolutions
   */
  private double getCancoderAbsolutePosition(CANcoder coder) {
    return coder.getAbsolutePosition().refresh().getValue();
  }
}

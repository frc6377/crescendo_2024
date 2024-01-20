package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private static final double maxSpeed = Units.feetToMeters(18.2); // Desired top speed
  private static final double maxAngularRate =
      Math.PI; // Max angular velocity in radians per second
  private final Telemetry telemetry = new Telemetry(maxSpeed);

  private static boolean isFieldOriented = true;
  private static Rotation2d alignmentRotation = null;

  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
    this.registerTelemetry(telemetry::telemeterize);
  }

  public CommandSwerveDrivetrain(
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

  // xSpeed, ySpeed, rotationSpeed should be axes with range -1<0<1
  public SwerveRequest getDriveRequest(double xSpeed, double ySpeed, double rotationSpeed) {
    xSpeed *= maxSpeed;
    ySpeed *= maxSpeed;
    rotationSpeed *= maxAngularRate;
    if (isFieldOriented) {
      if (alignmentRotation != null) {
        return new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(alignmentRotation)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed);
      }
      return new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withVelocityX(xSpeed)
          .withVelocityY(ySpeed)
          .withRotationalRate(rotationSpeed);
    }
    return new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(rotationSpeed);
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
}

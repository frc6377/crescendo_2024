package frc.robot.config;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import frc.robot.CommandSwerveDrivetrain;

public class TunerConstants {
  // Both sets of gains need to be tuned to your individual robot.

  // The steer motor uses any SwerveModule.SteerRequestType control request with
  // the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.05).withKS(0).withKV(1.5).withKA(0);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final double kSlipCurrentA = 300.0;

  // Theoretical free speed (m/s) at 12v applied output;
  // This needs to be tuned to your individual robot
  private static final double kSpeedAt12VoltsMps = 6.0;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 0;

  private static final double kDriveGearRatio = 1;
  private static final double kSteerGearRatio = 1;
  private static final double kWheelRadiusInches = 2;

  private static final boolean kSteerMotorReversed = false;
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final String kCANbusName = "";
  private static final int kPigeonId = 0;

  // These are only used for simulation
  private static final double kSteerInertia = 0.00001;
  private static final double kDriveInertia = 0.001;

  private static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);

  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(kSlipCurrentA)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withSteerMotorInverted(kSteerMotorReversed);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 1;
  private static final int kFrontLeftSteerMotorId = 5;
  private static final int kFrontLeftEncoderId = 1;
  private static final double kFrontLeftEncoderOffset = 0.25;

  private static final double kFrontLeftXPosInches = 10;
  private static final double kFrontLeftYPosInches = 10;

  // Front Right
  private static final int kFrontRightDriveMotorId = 2;
  private static final int kFrontRightSteerMotorId = 6;
  private static final int kFrontRightEncoderId = 2;
  private static final double kFrontRightEncoderOffset = 0.25;

  private static final double kFrontRightXPosInches = 10;
  private static final double kFrontRightYPosInches = -10;

  // Back Left
  private static final int kBackLeftDriveMotorId = 3;
  private static final int kBackLeftSteerMotorId = 7;
  private static final int kBackLeftEncoderId = 3;
  private static final double kBackLeftEncoderOffset = 0.25;

  private static final double kBackLeftXPosInches = -10;
  private static final double kBackLeftYPosInches = 10;

  // Back Right
  private static final int kBackRightDriveMotorId = 4;
  private static final int kBackRightSteerMotorId = 8;
  private static final int kBackRightEncoderId = 4;
  private static final double kBackRightEncoderOffset = 0.25;

  private static final double kBackRightXPosInches = -10;
  private static final double kBackRightYPosInches = -10;

  private final SwerveModuleConstants FrontLeft;
  private final SwerveModuleConstants FrontRight;
  private final SwerveModuleConstants BackLeft;
  private final SwerveModuleConstants BackRight;
  public final CommandSwerveDrivetrain drivetrain;

  protected TunerConstants(double frontLeftOffset, double frontRightOffset, double backLeftOffset, double backRightOffset) {
    FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId,
            kFrontLeftDriveMotorId,
            kFrontLeftEncoderId,
            frontLeftOffset,
            Units.inchesToMeters(kFrontLeftXPosInches),
            Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide);
    FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId,
            kFrontRightDriveMotorId,
            kFrontRightEncoderId,
            frontRightOffset,
            Units.inchesToMeters(kFrontRightXPosInches),
            Units.inchesToMeters(kFrontRightYPosInches),
            kInvertRightSide);
    BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId,
            kBackLeftDriveMotorId,
            kBackLeftEncoderId,
            backLeftOffset,
            Units.inchesToMeters(kBackLeftXPosInches),
            Units.inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide);
    BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId,
            kBackRightDriveMotorId,
            kBackRightEncoderId,
            backRightOffset,
            Units.inchesToMeters(kBackRightXPosInches),
            Units.inchesToMeters(kBackRightYPosInches),
            kInvertRightSide);
    drivetrain =
        new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
  }
}

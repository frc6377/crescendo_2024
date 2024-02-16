// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.config.DynamicRobotConfig;
import frc.robot.config.TurretZeroConfig;
import frc.robot.stateManagement.AllianceColor;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.HowdyMath;
import frc.robot.utilities.LimelightHelpers;
import java.util.function.Consumer;

public class TurretSubsystem extends SubsystemBase {

  private CANSparkMax turretMotor;
  private SingleJointedArmSim turretSim;
  private Mechanism2d turretMech;
  private MechanismRoot2d root;
  private MechanismLigament2d turretAngleSim;
  private ShuffleboardTab turretTab = Shuffleboard.getTab("Turret Tab");
  private SimDeviceSim simEncoder;
  private SimDouble simTurretPos;

  private PIDController turretPIDController;
  private CANcoder highGearCANcoder;
  private CANcoder lowGearCANcoder;
  private double turretPosition;
  private double turretVelocity;
  private Consumer<Double> testPosition;
  private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
  private GenericEntry kP = configTab.add("Turret KP", Constants.TurretConstants.KP).getEntry();
  private GenericEntry kI = configTab.add("Turret KI", Constants.TurretConstants.KI).getEntry();
  private GenericEntry kD = configTab.add("Turret KD", Constants.TurretConstants.KD).getEntry();
  private DebugEntry<Double> turretPositionEntry =
      new DebugEntry<Double>(turretPosition, "Position", this);
  private DebugEntry<Double> turretGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Goal Position", this);
  private DebugEntry<Double> turretVelocityEntry =
      new DebugEntry<Double>(turretVelocity, "Velocity", this);

  private DebugEntry<Double> tagDistanceEntry =
      new DebugEntry<Double>(0.0, "LastMeasuredTagDistance", this);

  private final RobotStateManager robotStateManager;
  private final VisionSubsystem visionSubsystem;

  public TurretSubsystem(RobotStateManager robotStateManager, VisionSubsystem visionSubsystem) {
    turretMotor = new CANSparkMax(Constants.TurretConstants.MOTOR_ID, MotorType.kBrushless);
    // Simulation
    if (Robot.isSimulation()) {
      turretSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1),
              1 / Constants.TurretConstants.TURRET_MOTOR_TURRET_RATIO,
              3.5 * 0.1016 * 0.1016 / 3,
              0.1016,
              -Math.toRadians(Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES),
              Math.toRadians(Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES),
              false,
              0);
      turretMech = new Mechanism2d(4, 4);
      root = turretMech.getRoot("Root", 2, 2);
      turretAngleSim =
          root.append(new MechanismLigament2d("Turret", 2, 0, 5, new Color8Bit(Color.kRed)));
      turretTab.add("Turret", turretMech);
    }

    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(40);

    this.robotStateManager = robotStateManager;
    this.visionSubsystem = visionSubsystem;

    turretMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse,
        (float)
            (-Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES
                / (360 * Constants.TurretConstants.TURRET_MOTOR_TURRET_RATIO)));
    turretMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward,
        (float)
            (Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES
                / (360 * Constants.TurretConstants.TURRET_MOTOR_TURRET_RATIO)));

    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    // initialze PID controller and encoder objects
    turretPIDController =
        new PIDController(
            Constants.TurretConstants.KP,
            Constants.TurretConstants.KI,
            Constants.TurretConstants.KD);

    highGearCANcoder = new CANcoder(Constants.TurretConstants.highGearCAN_CODER_ID);
    lowGearCANcoder = new CANcoder(Constants.TurretConstants.lowGearCAN_CODER_ID);
    TurretZeroConfig zeroConfig = new DynamicRobotConfig().getTurretZeroConfig();
    MagnetSensorConfigs highGearSensorConfigs =
        new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(zeroConfig.highGearTurretZero);
    MagnetSensorConfigs lowGearSensorConfigs =
        new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(zeroConfig.lowGearTurretZero);

    highGearCANcoder.getConfigurator().apply(highGearSensorConfigs);
    lowGearCANcoder.getConfigurator().apply(lowGearSensorConfigs);

    simEncoder =
        new SimDeviceSim(
            "CANEncoder:CANCoder (v6)", Constants.TurretConstants.highGearCAN_CODER_ID);
    simTurretPos = simEncoder.getDouble("rawPositionInput");

    turretPIDController.setIZone(Constants.TurretConstants.KIZ);

    zeroTurret();
    turretTab.add("zero turret", zeroZeroing());
  }

  private void stopTurret() {
    turretMotor.stopMotor();
  }

  public Command stowTurret() {
    return new InstantCommand(() -> setTurretPos(Math.toRadians(0))).withName("StowTurretCommand");
  }

  public Command zeroTurretCommand() {
    return Commands.runOnce(() -> this.zeroTurret(), this).withName("ZeroTurretCommand");
  }

  public double calculateTurretPosition() {
    double encoderPosition =
        lowGearCANcoder.getPosition().getValueAsDouble()
            / Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO;
    return encoderPosition + Constants.TurretConstants.ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV;
  }

  /**
   * A command to set the current turret position as true zero.
   *
   * @return a command that sets the current position as true zero
   */
  public Command zeroZeroing() {
    return Commands.runOnce(
        () -> {
          MagnetSensorConfigs cfg = new MagnetSensorConfigs();
          cfg.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
          cfg.withMagnetOffset(0);
          CANcoderConfigurator lowGearCANcoderConfigurator = lowGearCANcoder.getConfigurator();
          CANcoderConfigurator highGearCANcoderConfigurator = highGearCANcoder.getConfigurator();
          lowGearCANcoderConfigurator.apply(cfg);
          highGearCANcoderConfigurator.apply(cfg);

          final double trueZeroLowGearOffset =
              lowGearCANcoder.getAbsolutePosition().getValueAsDouble();
          final double trueZeroHighGearOffset =
              highGearCANcoder.getAbsolutePosition().getValueAsDouble();

          final double lowGearOffset =
              trueZeroLowGearOffset
                  - TurretConstants.LOW_GEAR_CAN_CODER_RATIO
                      * TurretConstants.ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV;
          final double highGearOffset =
              trueZeroHighGearOffset
                  - TurretConstants.HIGH_GEAR_CAN_CODER_RATIO
                      * TurretConstants.ENCODER_ZERO_OFFSET_FROM_TURRET_ZERO_REV;

          MagnetSensorConfigs newCfgLowGear = new MagnetSensorConfigs();
          newCfgLowGear.withMagnetOffset(lowGearOffset);
          lowGearCANcoderConfigurator.apply(newCfgLowGear);

          MagnetSensorConfigs newCfgHighGear = new MagnetSensorConfigs();
          newCfgHighGear.withMagnetOffset(highGearOffset);
          highGearCANcoderConfigurator.apply(newCfgHighGear);

          DynamicRobotConfig dynamicConfig = new DynamicRobotConfig();
          dynamicConfig.saveTurretZero(new TurretZeroConfig(lowGearOffset, highGearOffset));
        },
        this);
  }

  /** Will calculate the current turret position and update encoders and motors off of it. */
  public void zeroTurret() {
    double lowGearPosition = lowGearCANcoder.getAbsolutePosition().getValue().doubleValue();
    double highGearPosition = highGearCANcoder.getAbsolutePosition().getValue().doubleValue();
    Rotation2d turretRotation = encoderPositionsToTurretRotation(lowGearPosition, highGearPosition);

    lowGearCANcoder.setPosition(
        turretRotation.getRotations() * Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO);
    highGearCANcoder.setPosition(
        turretRotation.getRotations() * Constants.TurretConstants.HIGH_GEAR_CAN_CODER_RATIO);
    turretMotor
        .getEncoder()
        .setPosition(
            turretRotation.getRotations() * Constants.TurretConstants.TURRET_MOTOR_TURRET_RATIO);
  }

  /**
   * Calculates the turret zero off of given encoder rotations.
   *
   * @param lowGearCANcoderPosition The low gear encoder position, as in the gear with a lower gear
   *     ratio
   * @param highGearCANcoderPosition The high gear encoder position, as in the gear with a higher
   *     gear ratio
   * @return the calculated turret rotation
   */
  public static Rotation2d encoderPositionsToTurretRotation(
      double lowGearCANcoderPosition, double highGearCANcoderPosition) {

    // This equation is based off of
    // https://www.geeksforgeeks.org/implementation-of-chinese-remainder-theorem-inverse-modulo-based-implementation/
    // It is accurate to with in 3.6 deg
    int gearToothPosition =
        ((int) (lowGearCANcoderPosition * Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH)
                    * HowdyMath.inverse_modulus(
                        Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH,
                        Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH)
                    * Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH
                + (int)
                        (highGearCANcoderPosition
                            * Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH)
                    * HowdyMath.inverse_modulus(
                        Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH,
                        Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH)
                    * Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH)
            % (Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH
                * Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH);

    double roughRotation = gearToothPosition / (Constants.TurretConstants.TURRET_GEAR_TEETH + 0.0);
    double lowGearCANCoderDivsionSize =
        (Constants.TurretConstants.LOW_GEAR_CANCODER_TEETH + 0.0)
            / Constants.TurretConstants.TURRET_GEAR_TEETH;
    double highGearCANCoderDivsionSize =
        (Constants.TurretConstants.HIGH_GEAR_CANCODER_TEETH + 0.0)
            / Constants.TurretConstants.TURRET_GEAR_TEETH;

    double distToLowGearCanCoderDivide =
        Math.abs(0.5 - (roughRotation / lowGearCANCoderDivsionSize) % 1);
    double distToHighGearCanCoderDivide =
        Math.abs(0.5 - (roughRotation / highGearCANCoderDivsionSize) % 1);

    double position;
    if (distToLowGearCanCoderDivide < distToHighGearCanCoderDivide) {
      // use low gear CanCoder for fine zeroing
      position =
          fineTuneTurretRotation(
              roughRotation, lowGearCANCoderDivsionSize, lowGearCANcoderPosition);
    } else {
      // use high gear CanCoder for fine zeroing
      position =
          fineTuneTurretRotation(
              roughRotation, highGearCANCoderDivsionSize, highGearCANcoderPosition);
    }

    return Rotation2d.fromRotations(position);
  }

  private static double fineTuneTurretRotation(
      double roughPosition, double divisionSize, double CANCoderAngle) {
    int division = (int) ((roughPosition / divisionSize));
    return divisionSize * division + CANCoderAngle * divisionSize;
  }

  private void setTurretPos(double setpoint) {
    turretGoalPositionEntry.log(setpoint);
    turretMotor.set(
        -turretPIDController.calculate(
            turretPosition,
            MathUtil.clamp(
                setpoint,
                Math.toRadians(-Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES),
                Math.toRadians(Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES))));
  }

  private void holdPosition() {
    setTurretPos(turretPosition);
  }

  public Command idleTurret() {
    return run(() -> holdPosition()).withName("idleTurret");
  }

  private void updateTurretPosition() {
    turretPosition = calculateTurretPosition();
    SmartDashboard.putNumber("Turret Position", turretPosition);
    SmartDashboard.putBoolean("Out of Bounds", Math.abs(turretPosition) > 3.14);
    SmartDashboard.putBoolean(
        "Soft limit enabled forward", turretMotor.isSoftLimitEnabled(SoftLimitDirection.kForward));
    SmartDashboard.putBoolean(
        "Soft limit enabled reverse", turretMotor.isSoftLimitEnabled(SoftLimitDirection.kReverse));
  }

  public double getTurretPos() {
    return turretPosition; // returns the absolute encoder position in radians
  }

  public double getTurretVel() {
    return turretVelocity;
  }

  public Command testTurretCommand(double degrees) {
    return runEnd(() -> setTurretPos(Math.toRadians(degrees)), this::stopTurret)
        .withName("TestTurret");
  }

  private void aimTurret() {
    if (visionSubsystem != null) {
      int tagID =
          ((robotStateManager.getAllianceColor() == AllianceColor.BLUE)
              ? Constants.TurretConstants.SPEAKER_TAG_ID_BLUE
              : Constants.TurretConstants.SPEAKER_TAG_ID_RED);
      double visionTX = visionSubsystem.getTurretYaw(tagID);
      if (visionTX != 0) {
        // X & Rotation
        setTurretPos(Math.toRadians(visionTX) + turretPosition);

        // Y & Tilting
        double visionTY = visionSubsystem.getTurretPitch(tagID);
        double distanceToTag = tyToDistanceFromTag(visionTY);
        tagDistanceEntry.log(distanceToTag);
        // TODO: Add vertical tilt and use distance for it

        if (Math.abs(Math.toRadians(visionTX) + turretPosition)
            > Math.toRadians(Constants.TurretConstants.MAX_TURRET_ANGLE_DEGREES)) {
          // TODO: Make turret rotate the drivebase if necessary and driver thinks it's a good idea
        }
      }
    } else {
      // TODO: Make turret default to using odometry
      setTurretPos(turretPosition);
    }
  }

  private void aimTurretOdometry(Pose2d robotPos, Pose2d targetPos) {
    setTurretPos(getTurretRotationFromOdometry(robotPos, targetPos));
  }

  public Command getAimTurretCommand() {
    return run(() -> aimTurret()).withName("AimTurretCommand");
  }

  private double tyToDistanceFromTag(double ty) {
    double tagTheta =
        Math.toRadians(ty) + Constants.VisionConstants.ALPHABOT_LIMELIGHT_PITCH_RADIANS;
    double height =
        Constants.TurretConstants.SPEAKER_TAG_CENTER_HEIGHT_INCHES
            - Constants.VisionConstants
                .ALPHABOT_LIMELIGHT_Z_INCHES; // TODO: Change these alphabot constants to be turret
    // constants whenever the robot is built
    double distance = height / Math.tan(tagTheta);
    return distance;
  }

  @Override
  public void periodic() {
    turretPIDController.setP(kP.getDouble(turretPosition));
    turretPIDController.setI(kI.getDouble(turretPosition));
    turretPIDController.setD(kD.getDouble(turretPosition));
    updateTurretPosition();
    turretVelocity =
        (lowGearCANcoder.getVelocity().getValueAsDouble()
                * Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO)
            * 60; // changing from rotations per second to rotations per minute or rpm
    turretPositionEntry.log(turretPosition);
    turretVelocityEntry.log(turretVelocity);
  }

  @Override
  public void simulationPeriodic() {
    turretSim.setInput(turretMotor.get() * RobotController.getBatteryVoltage());
    turretSim.update(Robot.defaultPeriodSecs);
    turretAngleSim.setAngle(Math.toDegrees(turretSim.getAngleRads()));
    SmartDashboard.putNumber("Turret Angle", Math.toDegrees(turretSim.getAngleRads()));
    simTurretPos.set(
        Units.radiansToRotations(
            turretSim.getAngleRads() / Constants.TurretConstants.TURRET_MOTOR_TURRET_RATIO));
  }

  private double getTurretRotationFromOdometry(Pose2d robotPos, Pose2d targetPos) {
    return Math.atan2(robotPos.getY() - targetPos.getY(), robotPos.getX() - targetPos.getX())
        + robotPos.getRotation().getRadians();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turretSubsystem;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxSim;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.enabledSubsystems;
import frc.robot.Robot;
import frc.robot.config.TunerConstants;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.HowdyMath;
import java.util.function.DoubleSupplier;

public class TurretSubsystem extends SubsystemBase {
  private final CANSparkMax turretMotor;
  private final PIDController turretPositionPIDController;
  private DoubleSupplier positionErrorSupplier;

  private final CANSparkMaxSim pitchMotor;
  private final PIDController pitchPIDController;
  private final ArmFeedforward pitchFeedForward;

  private final CANcoder highGearCANcoder;
  private final CANcoder lowGearCANcoder;
  private final SparkAbsoluteEncoder pitchEncoder;
  private final AbsoluteEncoder pitchAbsoluteEncoder;

  private double turretPosition = 0;
  private double turretVelocity;
  private double pitchPosition = 10;
  private double pitchVelocity;

  private SingleJointedArmSim turretSim;
  private Mechanism2d turretMech;
  private MechanismRoot2d turretRoot;
  private MechanismLigament2d turretAngleSim;

  private SingleJointedArmSim pitchSim;
  private Mechanism2d pitchMech;
  private MechanismRoot2d pitchRoot;
  private MechanismLigament2d pitchAngleSim;

  private SimDeviceSim simTurretEncoder;
  private SimDouble simTurretPos;

  private final ShuffleboardTab turretTab = Shuffleboard.getTab(this.getName());
  private final DebugEntry<Double> turretPositionEntry =
      new DebugEntry<Double>(turretPosition, "Turret Position (Degrees)", this);
  private final DebugEntry<Double> turretGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Turret Goal Position (Degrees)", this);
  private final DebugEntry<Double> turretVelocityEntry =
      new DebugEntry<Double>(turretVelocity, "Turret Velocity (RPM)", this);
  private final DebugEntry<Double> pitchPositionEntry =
      new DebugEntry<Double>(pitchPosition, "Pitch Position (Degrees)", this);
  private final DebugEntry<Double> pitchGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Pitch Goal Position (Degrees)", this);
  private final DebugEntry<Double> pitchVelocityEntry =
      new DebugEntry<Double>(pitchVelocity, "Pitch Velocity (RPM)", this);
  private final DebugEntry<Double> motorOutputEntry =
      new DebugEntry<Double>(0.0, "Turret Motor Output", this);
  private DebugEntry<String> currentCommand =
      new DebugEntry<String>("none", "Turret Command", this);
  private DebugEntry<Double> pitchMotorOutput =
      new DebugEntry<Double>(0.0, "Pitch Motor Output", this);
  private DebugEntry<Double> positionErrorLog =
      new DebugEntry<Double>(0.0, "Position Error (deg?)", this);
  private DebugEntry<Double> targetVelocityLog =
      new DebugEntry<Double>(0.0, "Target Velocity (RPM)", this);

  private DebugEntry<Boolean> isLastChanceLog =
      new DebugEntry<Boolean>(false, "isLastChance", this);

  private boolean usingPitchPid = true;
  private final PIDController turretVelocityPIDController;
  private boolean isLastChance;

  public TurretSubsystem(RobotStateManager robotStateManager, VisionSubsystem visionSubsystem) {
    this.positionErrorSupplier = () -> 0;

    // Initialize Motors
    turretMotor = new CANSparkMax(Constants.TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setIdleMode(IdleMode.kBrake);
    turretMotor.setInverted(Constants.TurretConstants.IS_MOTOR_INVERTED);
    turretMotor.setSmartCurrentLimit(Constants.TurretConstants.TURRET_SMART_CURRENT_LIMIT);
    turretMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse,
        (float) TurretConstants.TURRET_MIN_ANGLE_ROTATIONS);
    turretMotor.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward,
        (float) TurretConstants.TURRET_MAX_ANGLE_ROTATIONS);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    turretMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    turretPositionPIDController = TurretConstants.TURRET_POSITION_PID_CASCADE.getPIDController();
    turretPositionPIDController.setSetpoint(0);
    TurretConstants.TURRET_POSITION_PID_CASCADE.createTunableNumbers(
        "Turret Motor", turretPositionPIDController, this);
    turretVelocityPIDController = TurretConstants.TURRET_VELOCITY_PID_CASCADE.getPIDController();
    turretTab.addFloat("rotation current", () -> (float) turretMotor.getOutputCurrent());
    turretTab.addFloat("rotation temp", () -> (float) turretMotor.getMotorTemperature());

    pitchMotor = new CANSparkMaxSim(Constants.TurretConstants.PITCH_MOTOR_ID, MotorType.kBrushless);
    pitchMotor.restoreFactoryDefaults();
    pitchMotor.setSmartCurrentLimit(Constants.TurretConstants.PITCH_SMART_CURRENT_LIMIT);
    pitchMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    pitchAbsoluteEncoder = pitchMotor.getAbsoluteEncoder();
    pitchAbsoluteEncoder.setPositionConversionFactor(1);
    pitchAbsoluteEncoder.setZeroOffset(TurretConstants.PITCH_ZERO_OFFSET);
    pitchAbsoluteEncoder.setInverted(true);
    pitchMotor.setIdleMode(IdleMode.kCoast);

    pitchPIDController = TurretConstants.PITCH_PID.getPIDController();
    TurretConstants.PITCH_PID.createTunableNumbers("Pitch Motor", pitchPIDController, this);
    pitchFeedForward = TurretConstants.PITCH_FF.getArmFeedforward();

    // Encoders
    highGearCANcoder = new CANcoder(Constants.TurretConstants.highGearCAN_CODER_ID);
    lowGearCANcoder = new CANcoder(Constants.TurretConstants.lowGearCAN_CODER_ID);
    pitchEncoder = pitchMotor.getAbsoluteEncoder();

    MagnetSensorConfigs highGearSensorConfigs =
        new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(TurretConstants.TurretZeroConfig.highGearTurretZero);
    MagnetSensorConfigs lowGearSensorConfigs =
        new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(-0.709473 - 0.5);

    highGearCANcoder.getConfigurator().apply(highGearSensorConfigs);
    lowGearCANcoder.getConfigurator().apply(lowGearSensorConfigs);

    isLastChance = false;

    // Simulation
    if (Robot.isSimulation()) {
      // Turret
      simTurretEncoder =
          new SimDeviceSim(
              "CANEncoder:CANCoder (v6)", Constants.TurretConstants.highGearCAN_CODER_ID);
      simTurretPos = simTurretEncoder.getDouble("rawPositionInput");
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
      turretRoot = turretMech.getRoot("Root", 2, 2);
      turretAngleSim =
          turretRoot.append(new MechanismLigament2d("Turret", 2, 0, 5, new Color8Bit(Color.kRed)));
      turretTab.add("Turret", turretMech);

      // pitch
      pitchSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1),
              Constants.TurretConstants.PITCH_CONVERSION_FACTOR,
              Constants.TurretConstants.SHOOTER_MASS
                  * Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY
                  * Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY
                  / 3,
              Constants.TurretConstants.SHOOTER_CENTER_OF_GRAVITY
                  / TurretConstants.SIMULATION_CG_MAGIC_NUMBER,
              Math.toRadians(Constants.TurretConstants.PITCH_MIN_ANGLE_DEGREES),
              Math.toRadians(Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES),
              true,
              0);
      pitchMech = new Mechanism2d(4, 4);
      pitchRoot = pitchMech.getRoot("Root", 2, 2);
      pitchAngleSim =
          pitchRoot.append(new MechanismLigament2d("Pitch", 2, 0, 5, new Color8Bit(Color.kBlue)));
      turretTab.add("Pitch", pitchMech);
    }
  }

  public void stopTurret() {
    turretMotor.stopMotor();
    pitchMotor.stopMotor();
  }

  public double calculateTurretPosition() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return 0;
    double encoderPosition =
        (lowGearCANcoder.getPosition().getValueAsDouble() - 0.5)
            / Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO;
    return encoderPosition;
  }

  private double calculatePitchPosition() {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return 0;
    double pos = pitchEncoder.getPosition();
    if (pos > 0.5) {
      pos -= 1;
    }
    return Units.rotationsToRadians(pos);
  }

  /** Will calculate the current turret position and update encoders and motors off of it. */
  public void zeroTurret() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return;
    Rotation2d turretRotation = new Rotation2d();

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

  public void setTurretPos(double setpoint) {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return;
    double limitedSetpoint =
        MathUtil.clamp(
            setpoint,
            Math.toRadians(Constants.TurretConstants.TURRET_MIN_ANGLE_DEGREES),
            Math.toRadians(Constants.TurretConstants.TURRET_MAX_ANGLE_DEGREES));
    turretGoalPositionEntry.log(Units.radiansToDegrees(setpoint));
    this.setPositionErrorSupplier(() -> turretPosition - Units.radiansToRotations(limitedSetpoint));
  }

  /**
   * Sets the method to calculate the current turret angle. Is expected in rotaions.
   *
   * @param positionErrorSupplier the method to call
   */
  public void setPositionErrorSupplier(DoubleSupplier positionErrorSupplier) {
    this.positionErrorSupplier = positionErrorSupplier;
  }

  public void setPitchPos(double setpoint) {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return;
    usingPitchPid = true;
    pitchPIDController.setSetpoint(
        MathUtil.clamp(
            setpoint,
            Math.toRadians(Constants.TurretConstants.PITCH_MIN_ANGLE_DEGREES),
            Math.toRadians(Constants.TurretConstants.PITCH_MAX_ANGLE_DEGREES)));
    pitchGoalPositionEntry.log(Math.toDegrees(setpoint));
  }

  public void holdPosition() {
    if (Constants.enabledSubsystems.turretRotationEnabled) setTurretPos(0);
    if (Constants.enabledSubsystems.turretPitchEnabled) {
      usingPitchPid = false;
      pitchMotor.setVoltage(-0.1);
    }
  }

  public void updateTurretPosition() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return;
    turretPosition = calculateTurretPosition();
    turretVelocity =
        (lowGearCANcoder.getVelocity().getValueAsDouble()
                / Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO)
            * 60; // changing from rotations per second to rotations per minute or rpm
  }

  private void updatePitchPosition() {
    if (!Constants.enabledSubsystems.turretPitchEnabled) return;
    pitchPosition = calculatePitchPosition();
  }

  public double getTurretPos() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return 0;
    return turretPosition; // returns the absolute encoder position in radians
  }

  public double getTurretVel() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return 0;
    return turretVelocity;
  }

  @Override
  public void periodic() {
    if (enabledSubsystems.turretRotationEnabled) {
      SmartDashboard.putNumber("Pigeon Angle", TunerConstants.drivetrain.getPigeon2().getAngle());
      updateTurretPosition();
      double positionError = positionErrorSupplier.getAsDouble();
      positionErrorLog.log(positionError);

      final double targetVelocity = turretPositionPIDController.calculate(positionError);
      final double motorOut = turretVelocityPIDController.calculate(getTurretVel(), targetVelocity);
      targetVelocityLog.log(targetVelocity);
      turretMotor.set(motorOut);

      turretPositionEntry.log(Units.rotationsToDegrees(turretPosition));
      turretVelocityEntry.log(turretVelocity);
      motorOutputEntry.log(turretMotor.get());
    }

    if (enabledSubsystems.turretPitchEnabled) {
      updatePitchPosition();
      if (usingPitchPid) {
        pitchMotor.setVoltage(
            pitchPIDController.calculate(pitchPosition)
                + pitchFeedForward.calculate(pitchPosition, 0));
      }
      pitchVelocity =
          pitchEncoder.getVelocity()
              * 60; // changing from rotations per second to rotations per minute or rpm
      pitchPositionEntry.log(Units.radiansToDegrees(pitchPosition));
      pitchVelocityEntry.log(pitchVelocity);
      pitchMotorOutput.log(pitchMotor.getAppliedOutput());
    }

    if (this.getCurrentCommand() != null) currentCommand.log(this.getCurrentCommand().getName());
  }

  @Override
  public void simulationPeriodic() {
    turretSim.setInput(turretMotor.get() * RobotController.getBatteryVoltage());
    turretSim.update(Robot.defaultPeriodSecs);
    turretAngleSim.setAngle(Math.toDegrees(turretSim.getAngleRads()));
    simTurretPos.set(
        Units.radiansToRotations(
            turretSim.getAngleRads() * Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO));

    pitchSim.setInput(pitchMotor.getAppliedOutput());
    pitchSim.update(Robot.defaultPeriodSecs);
    pitchAngleSim.setAngle(Math.toDegrees(pitchSim.getAngleRads()));
    pitchMotor.setAbsolutePosition(Units.radiansToRotations(pitchSim.getAngleRads()));
  }

  private double getTurretRotationFromOdometry(Pose2d robotPos, Pose2d targetPos) {
    return Math.atan2(robotPos.getY() - targetPos.getY(), robotPos.getX() - targetPos.getX())
        + robotPos.getRotation().getRadians();
  }

  protected CANcoder getLowGearCANCoder() {
    return lowGearCANcoder;
  }

  protected CANcoder getHighGearCANCoder() {
    return highGearCANcoder;
  }

  /**
   * The elevation of the turret in radians
   *
   * @return the current elevation of the turret
   */
  public double getPitch() {
    return pitchPosition;
  }

  /**
   * Checks if the turret is at its current set point +/- epsilion
   *
   * @param epsilion the allowed error margin
   * @return if the turret is at its setpoint
   */
  public boolean turretAtSetPoint(Rotation2d epsilion) {
    return Math.abs(positionErrorSupplier.getAsDouble()) < epsilion.getRotations();
  }

  DebugEntry<Boolean> atPitchLog = new DebugEntry<Boolean>(false, "Pitch as setpoint", this);

  public boolean pitchAtSetpoint() {
    pitchPIDController.setTolerance(TurretConstants.PITCH_TOLERANCE);
    boolean result = pitchPIDController.atSetpoint();
    atPitchLog.log(result);
    return result;
  }

  public void setLastChance() {
    isLastChance = true;
    isLastChanceLog.log(isLastChance);
  }

  public void resetLastChance() {
    isLastChance = false;
    isLastChanceLog.log(isLastChance);
  }

  public boolean getLastChance() {
    return isLastChance;
  }
}

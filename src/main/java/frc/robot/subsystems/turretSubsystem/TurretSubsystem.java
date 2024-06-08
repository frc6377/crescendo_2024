// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turretSubsystem;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.enabledSubsystems;
import frc.robot.Robot;
import frc.robot.stateManagement.RobotStateManager;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.HowdyMath;
import java.util.function.DoubleSupplier;

public class TurretSubsystem extends SubsystemBase {
  private final RobotStateManager RSM;

  private final CANSparkMax turretMotor;
  private final PIDController turretPositionPIDController;
  private DoubleSupplier positionErrorSupplier;

  private final CANcoder highGearCANcoder;
  private final CANcoder lowGearCANcoder;

  private double turretPosition = 0;
  private double turretVelocity;

  private SingleJointedArmSim turretSim;
  private Mechanism2d turretMech;
  private MechanismRoot2d turretRoot;
  private MechanismLigament2d turretAngleSim;

  private SimDeviceSim simTurretEncoder;
  private SimDouble simTurretPos;

  private final ShuffleboardTab turretTab = Shuffleboard.getTab(this.getName());
  private final DebugEntry<Double> turretPositionEntry =
      new DebugEntry<Double>(turretPosition, "Turret Position (Degrees)", this);
  private final DebugEntry<Double> turretGoalPositionEntry =
      new DebugEntry<Double>(0.0, "Turret Goal Position (Degrees)", this);
  private final DebugEntry<Double> turretVelocityEntry =
      new DebugEntry<Double>(turretVelocity, "Turret Velocity (RPM)", this);
  private final DebugEntry<Double> motorOutputEntry =
      new DebugEntry<Double>(0.0, "Turret Motor Output", this);
  private DebugEntry<String> currentCommand =
      new DebugEntry<String>("none", "Turret Command", this);
  private DebugEntry<Double> positionErrorLog =
      new DebugEntry<Double>(0.0, "Position Error (deg?)", this);
  private DebugEntry<Double> targetVelocityLog =
      new DebugEntry<Double>(0.0, "Target Velocity (RPM)", this);

  private final PIDController turretVelocityPIDController;

  public TurretSubsystem(RobotStateManager robotStateManager, VisionSubsystem visionSubsystem) {
    this.positionErrorSupplier = () -> 0;
    RSM = robotStateManager;

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

    // Encoders
    highGearCANcoder = new CANcoder(Constants.TurretConstants.highGearCAN_CODER_ID);
    lowGearCANcoder = new CANcoder(Constants.TurretConstants.lowGearCAN_CODER_ID);

    MagnetSensorConfigs highGearSensorConfigs =
        new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(TurretConstants.TurretZeroConfig.highGearTurretZero);
    MagnetSensorConfigs lowGearSensorConfigs =
        new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(0.509473);

    highGearCANcoder.getConfigurator().apply(highGearSensorConfigs);
    lowGearCANcoder.getConfigurator().apply(lowGearSensorConfigs);

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
    }
  }

  public void stopTurret() {
    turretMotor.stopMotor();
  }

  /**
   * Calculates the turret rotation in radians
   *
   * @return
   */
  public double calculateTurretPosition() {
    if (!Constants.enabledSubsystems.turretRotationEnabled) return 0;
    double encoderPosition =
        (lowGearCANcoder.getPosition().getValueAsDouble() - 0.5)
            / Constants.TurretConstants.LOW_GEAR_CAN_CODER_RATIO;
    return encoderPosition;
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

  public void holdPosition() {
    if (Constants.enabledSubsystems.turretRotationEnabled) {
      setTurretPos(0);
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
      updateTurretPosition();
      double positionError = positionErrorSupplier.getAsDouble();
      positionErrorLog.log(positionError);

      final double targetVelocity = turretPositionPIDController.calculate(positionError);
      final double motorOut = turretVelocityPIDController.calculate(getTurretVel(), targetVelocity);
      targetVelocityLog.log(targetVelocity);
      turretMotor.set(motorOut);

      RSM.setTurretRotation(Rotation2d.fromRotations(calculateTurretPosition()));

      turretPositionEntry.log(Units.rotationsToDegrees(turretPosition));
      turretVelocityEntry.log(turretVelocity);
      motorOutputEntry.log(turretMotor.get());
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
   * Checks if the turret is at its current set point +/- epsilion
   *
   * @param epsilion the allowed error margin
   * @return if the turret is at its setpoint
   */
  public boolean turretAtSetPoint(Rotation2d epsilion) {
    return Math.abs(positionErrorSupplier.getAsDouble()) < epsilion.getRotations();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.trapElvSubsystem;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxSim;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapElvConstants;
import frc.robot.Robot;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.TOFSensorSimple;
import java.util.function.BooleanSupplier;

public class TrapElvSubsystem extends SubsystemBase {
  private final boolean isElv = false;

  // Wrist motors
  private final CANSparkMaxSim wristMotor;
  private double wristStateGoal;
  private final PIDController wristPIDController;
  private final ArmFeedforward wristFeedforward;

  // Roller Motor
  private final CANSparkMax rollerMotor;

  // Elevator motors
  private CANSparkMaxSim baseMotor1;
  private CANSparkMax baseMotor2;
  private CANSparkMaxSim scoringMotor;

  // Offsets
  private double baseMotorOffset1;
  private double baseMotorOffset2;
  private double scoringMotorOffset;

  // Beam Breaks
  private final TOFSensorSimple sourceBreak;
  private final DigitalInput groundBreak;

  // Limit Switches
  private DigitalInput baseLimit;
  private DigitalInput scoringLimit;

  // Encoders
  private final SparkAbsoluteEncoder wristEncoder;

  private DebugEntry<Boolean> sourceLog;
  private DebugEntry<Boolean> groundLog;
  private DebugEntry<Boolean> baseLog;
  private DebugEntry<Boolean> scoringLog;

  private Mechanism2d elvMechanism;
  private MechanismRoot2d root;
  private MechanismLigament2d baseMech;
  private MechanismLigament2d scoringMech;
  private MechanismLigament2d wristMech;

  private ElevatorSim m_baseElevatorSim;
  private ElevatorSim m_scoringElevatorSim;
  private SingleJointedArmSim m_wristMotorSim;

  private ShuffleboardTab TrapElvTab = Shuffleboard.getTab(this.getName());

  private DebugEntry<Double> currentPositionEntry;
  private DebugEntry<Boolean> isWristRollerRunning;
  private DebugEntry<Double> FFOutput;
  private DebugEntry<Double> wristOutput;
  private DebugEntry<Double> wristGoal;
  private DebugEntry<String> currentWristStateEntry;

  private double FF;

  // States
  public enum TrapElvState {
    // Degrees, elv height, elv height
    STOWED(-0.18, 0.0, 0.0),
    FROM_INTAKE(-0.18, 0.0, 0.0),
    FROM_SOURCE(0.4, 0.0, 12.0),
    AMP_SCORE(0.44, 0.0, 12.0);

    private double wristPose;
    private double basePose;
    private double scoringPose;

    TrapElvState(double wrist, double base, double scoring) {
      this.wristPose = wrist;
      this.basePose = base;
      this.scoringPose = scoring;
    }

    private double feetToRotations(double f) {
      return Units.inchesToMeters(f)
          / (2 * Math.PI * TrapElvConstants.DRUM_RADIUS)
          * TrapElvConstants.ELV_GEAR_RATIO;
    }

    public Double getWristPose() {
      return wristPose;
    }

    public Double getBasePose() {
      return feetToRotations(basePose);
    }

    public Double getScoringPose() {
      return feetToRotations(scoringPose);
    }
  }

  private TrapElvState currentWristState;

  /** Creates a new TrapArm. */
  public TrapElvSubsystem() {
    // Wrist
    wristMotor = new CANSparkMaxSim(TrapElvConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    wristMotor.clearFaults();

    wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    wristPIDController = TrapElvConstants.WRIST_PID.getPIDController();
    TrapElvConstants.WRIST_PID.createTunableNumbers("Wrist Motor", wristPIDController, this);

    wristFeedforward = TrapElvConstants.WRIST_FF.getArmFeedforward();

    wristStateGoal = TrapElvState.STOWED.getWristPose();

    wristEncoder = wristMotor.getAbsoluteEncoder();
    wristEncoder.setPositionConversionFactor(1);
    wristEncoder.setInverted(false);
    wristEncoder.setZeroOffset(TrapElvConstants.WRIST_ZERO_OFFSET);

    rollerMotor = new CANSparkMax(TrapElvConstants.ROLLER_MOTOR_ID, MotorType.kBrushed);
    rollerMotor.restoreFactoryDefaults();

    sourceBreak =
        new TOFSensorSimple(TrapElvConstants.SOURCE_BREAK_ID, TrapElvConstants.BREAK_THRESHOLD_MM);
    groundBreak = new DigitalInput(TrapElvConstants.GROUND_BREAK_ID);

    currentWristState = TrapElvState.STOWED;

    // Elv
    if (isElv) {
      baseLimit = new DigitalInput(TrapElvConstants.BASE_BREAK_ID);
      scoringLimit = new DigitalInput(TrapElvConstants.SCORING_BREAK_ID);
      baseLog = new DebugEntry<Boolean>(baseLimit.get(), "Base Limit Switch", this);
      scoringLog = new DebugEntry<Boolean>(scoringLimit.get(), "Scoring Limit Switch", this);

      baseMotorOffset1 = 0.0;
      baseMotorOffset2 = 0.0;

      baseMotor1 = new CANSparkMaxSim(TrapElvConstants.BASE_MOTOR1_ID, MotorType.kBrushless);
      baseMotor1.restoreFactoryDefaults();
      baseMotor1.getPIDController().setP(TrapElvConstants.BASE_PID[0]);
      baseMotor1.getPIDController().setI(TrapElvConstants.BASE_PID[1]);
      baseMotor1.getPIDController().setD(TrapElvConstants.BASE_PID[2]);
      baseMotor1.getPIDController().setIZone(TrapElvConstants.BASE_PID[3]);
      baseMotor1.getPIDController().setFF(TrapElvConstants.BASE_PID[4]);

      baseMotor2 = new CANSparkMax(TrapElvConstants.BASE_MOTOR2_ID, MotorType.kBrushless);
      baseMotor2.restoreFactoryDefaults();
      baseMotor2.getPIDController().setP(TrapElvConstants.BASE_PID[0]);
      baseMotor2.getPIDController().setI(TrapElvConstants.BASE_PID[1]);
      baseMotor2.getPIDController().setD(TrapElvConstants.BASE_PID[2]);
      baseMotor2.getPIDController().setIZone(TrapElvConstants.BASE_PID[3]);
      baseMotor2.getPIDController().setFF(TrapElvConstants.BASE_PID[4]);

      scoringMotor = new CANSparkMaxSim(TrapElvConstants.SCORING_MOTOR_ID, MotorType.kBrushless);
      scoringMotor.restoreFactoryDefaults();
      scoringMotor.getPIDController().setP(TrapElvConstants.SCORING_PID[0]);
      scoringMotor.getPIDController().setI(TrapElvConstants.SCORING_PID[1]);
      scoringMotor.getPIDController().setD(TrapElvConstants.SCORING_PID[2]);
      scoringMotor.getPIDController().setIZone(TrapElvConstants.SCORING_PID[3]);
      scoringMotor.getPIDController().setFF(TrapElvConstants.SCORING_PID[4]);
    }

    // Simulation
    if (Robot.isSimulation()) {
      // 2D Mechanism
      elvMechanism = new Mechanism2d(2, 2);
      root = elvMechanism.getRoot("Root", 1, 0);
      baseMech =
          root.append(
              new MechanismLigament2d(
                  "Base Elv", TrapElvConstants.ELV_MIN_HEIGHT, 90, 20, new Color8Bit(Color.kBlue)));
      scoringMech =
          baseMech.append(
              new MechanismLigament2d(
                  "Scoring Elv",
                  TrapElvConstants.ELV_MIN_HEIGHT,
                  0,
                  15,
                  new Color8Bit(Color.kAqua)));
      wristMech =
          scoringMech.append(
              new MechanismLigament2d(
                  "Wrist Mech", TrapElvConstants.WRIST_LENGTH, 0, 10, new Color8Bit(Color.kRed)));

      m_baseElevatorSim =
          new ElevatorSim(
              DCMotor.getNEO(2),
              TrapElvConstants.ELV_GEAR_RATIO,
              TrapElvConstants.ELV_LIFT_MASS,
              TrapElvConstants.DRUM_RADIUS,
              TrapElvConstants.ELV_MIN_HEIGHT,
              TrapElvConstants.ELV_MAX_HEIGHT,
              true,
              0);

      m_scoringElevatorSim =
          new ElevatorSim(
              DCMotor.getNEO(1),
              TrapElvConstants.ELV_GEAR_RATIO,
              TrapElvConstants.ELV_LIFT_MASS,
              TrapElvConstants.DRUM_RADIUS,
              TrapElvConstants.ELV_MIN_HEIGHT,
              TrapElvConstants.ELV_MAX_HEIGHT,
              true,
              0);

      m_wristMotorSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(1),
              TrapElvConstants.WRIST_GEAR_RATIO,
              TrapElvConstants.WRIST_MOI,
              TrapElvConstants.WRIST_LENGTH,
              TrapElvConstants.WRIST_MIN_ANGLE, // min rotation
              TrapElvConstants.WRIST_MAX_ANGLE, // max rotation
              true,
              0);

      TrapElvTab.add("Trap Arm Mech", elvMechanism).withPosition(7, 7);
    }

    sourceLog = new DebugEntry<Boolean>(sourceBreak.get(), "Source Beam Break", this);
    groundLog = new DebugEntry<Boolean>(groundBreak.get(), "Ground Beam Break", this);
    currentPositionEntry = new DebugEntry<>(getWristEncoderPos(), "Current wrist Position", this);
    isWristRollerRunning = new DebugEntry<Boolean>(false, "Wrist Rollers", this);
    FFOutput = new DebugEntry<Double>(0.0, "FF Output", this);
    wristOutput = new DebugEntry<Double>(0.0, "Wrist Motor Output", this);
    wristOutput = new DebugEntry<Double>(0.0, "Wrist Motor Output", this);
    wristGoal = new DebugEntry<Double>(wristStateGoal, "Wrist Goal", this);
    currentWristStateEntry =
        new DebugEntry<String>(TrapElvState.STOWED.name(), "Current Wrist State", this);
  }

  // Boolean Suppliers
  public BooleanSupplier getSourceBreak() {

    return () -> sourceBreak.get();
  }

  public BooleanSupplier getGroundBreak() {

    return () -> groundBreak.get();
  }

  public BooleanSupplier getBaseLimit() {

    return () -> baseLimit.get();
  }

  public BooleanSupplier getScoringLimit() {

    return () -> scoringLimit.get();
  }

  // Getter Functions
  private double getWristEncoderPos() {
    if (Robot.isReal()) {
      double a = wristEncoder.getPosition();
      if (a >= .75) {
        a = a - 1;
      }
      return a;
    }
    return Units.radiansToRotations(m_wristMotorSim.getAngleRads());
  }

  public BooleanSupplier isAMPReady() {
    double offset = wristStateGoal - getWristEncoderPos();
    return () ->
        (-TrapElvConstants.WRIST_DEADZONE < offset && offset < TrapElvConstants.WRIST_DEADZONE);
  }

  // Void Functions
  public void setWristState(TrapElvState state) {
    currentWristState = state;
    currentWristStateEntry.log(currentWristState.name());
    wristStateGoal = state.getWristPose();
    wristGoal.log(wristStateGoal);
    if (isElv) {
      baseMotor1
          .getPIDController()
          .setReference(state.getBasePose() - baseMotorOffset1, ControlType.kPosition);
      baseMotor2
          .getPIDController()
          .setReference(state.getBasePose() - baseMotorOffset2, ControlType.kPosition);
      scoringMotor
          .getPIDController()
          .setReference(state.getScoringPose() - scoringMotorOffset, ControlType.kPosition);
    }
  }

  public void stowTrapElv() {
    setWristState(TrapElvState.STOWED);
    setRoller(0);
  }

  public void setTrapArm(TrapElvState state) {
    if (isElv) {
      baseMotor1
          .getPIDController()
          .setReference(state.getBasePose() - baseMotorOffset1, ControlType.kPosition);
      baseMotor2
          .getPIDController()
          .setReference(state.getBasePose() - baseMotorOffset2, ControlType.kPosition);
      scoringMotor
          .getPIDController()
          .setReference(state.getScoringPose() - scoringMotorOffset, ControlType.kPosition);
    }
  }

  @Override
  public void periodic() {
    currentPositionEntry.log(getWristEncoderPos());
    sourceLog.log(sourceBreak.get());
    groundLog.log(groundBreak.get());
    sourceBreak.getMilliMeters();

    FF = wristFeedforward.calculate(Units.rotationsToRadians(getWristEncoderPos()), 0);

    wristMotor.setVoltage(
        MathUtil.clamp(
            wristPIDController.calculate(
                    Units.rotationsToDegrees(getWristEncoderPos()),
                    Units.rotationsToDegrees(wristStateGoal))
                + FF,
            -12,
            12));
    FFOutput.log(FF);
    wristOutput.log(wristMotor.getAppliedOutput());

    if (isElv) {
      baseLog.log(baseLimit.get());
      scoringLog.log(sourceBreak.get());
    }
  }

  @Override
  public void simulationPeriodic() {
    // Wrist Sim Stuff
    m_wristMotorSim.setInput(wristMotor.getAppliedOutput());
    m_wristMotorSim.update(Robot.defaultPeriodSecs);
    wristMotor.setAbsolutePosition(Units.radiansToRotations(m_wristMotorSim.getAngleRads()));
    // Offest added so that gravity is simulated in the right direction
    wristMech.setAngle(Units.radiansToDegrees(m_wristMotorSim.getAngleRads()) - 90);
    SmartDashboard.putNumber("Current Draw Wrist (A)", m_wristMotorSim.getCurrentDrawAmps());

    if (isElv) {
      for (double i = 0; i < Robot.defaultPeriodSecs; i += CANSparkMaxSim.kPeriod) {
        m_baseElevatorSim.setInput(baseMotor1.get() * RobotController.getBatteryVoltage());
        m_baseElevatorSim.update(CANSparkMaxSim.kPeriod);
        baseMotor1.update(
            m_baseElevatorSim.getVelocityMetersPerSecond()
                * TrapElvConstants.ELV_GEAR_RATIO
                / TrapElvConstants.DRUM_RADIUS);

        m_scoringElevatorSim.setInput(scoringMotor.get() * RobotController.getBatteryVoltage());
        m_scoringElevatorSim.update(CANSparkMaxSim.kPeriod);
        scoringMotor.update(
            m_scoringElevatorSim.getVelocityMetersPerSecond()
                * TrapElvConstants.ELV_GEAR_RATIO
                / TrapElvConstants.DRUM_RADIUS);
      }

      SmartDashboard.putNumber("base CAN Sim", baseMotor1.get());
      SmartDashboard.putNumber("scoring CAN Sim", scoringMotor.get());

      baseMech.setLength(m_baseElevatorSim.getPositionMeters());
      scoringMech.setLength(m_scoringElevatorSim.getPositionMeters());

      // baseElvLength.setDouble(Units.metersToInches(m_baseElevatorSim.getPositionMeters()));
      // scoringElvLength.setDouble(Units.metersToInches(m_scoringElevatorSim.getPositionMeters()));
    }
  }

  public void setRoller(double speed) {
    rollerMotor.set(speed);
    isWristRollerRunning.log(speed != 0);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxSim;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapElvConstants;
import frc.robot.Robot;
import frc.robot.utilities.DebugEntry;
import java.util.function.BooleanSupplier;

public class TrapElvSubsystem extends SubsystemBase {
  private final boolean isElv = false;
  // TODO: position, zero wrist, encoder logging, change pids
  // Wrist motors
  private final CANSparkMaxSim wristMotor;
  private double wristState;
  private final PIDController wristPIDController;
  private final ArmFeedforward wristFeedforward;

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
  private final DigitalInput sourceBreak;
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

  private DebugEntry<Double> offsetEntry;
  private DebugEntry<Double> currentPositionEntry;

  private ShuffleboardTab TrapElvTab = Shuffleboard.getTab("TrapElvSubsystem");
  private GenericEntry baseGoal = TrapElvTab.add("Base Goal", 0).withPosition(9, 0).getEntry();
  private GenericEntry FFOutput = TrapElvTab.add("FF Output", 0).withPosition(8, 0).getEntry();
  private GenericEntry wristOutput =
      TrapElvTab.add("Wrist Motor Output", 0).withPosition(7, 0).getEntry();
  private GenericEntry wristGoal = TrapElvTab.add("Wrist Goal", 0).withPosition(6, 0).getEntry();
  private GenericEntry wristEncoderOutput = TrapElvTab.add("Wrist Encoder Output", 0).withSize(2, 1).getEntry();

  private double FF;

  // States
  public static enum TrapElvState {
    // Degrees, elv height, elv height
    STOWED(-0.25, 0.0, 0.0),
    FROM_INTAKE(-0.25, 0.0, 0.0),
    FROM_SOURCE(-0.1, 0.0, 12.0),
    TRAP_SCORE(0.0, 12.0, 12.0),
    AMP_SCORE(-0.2, 0.0, 12.0);

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

  /** Creates a new TrapArm. */
  public TrapElvSubsystem() {
    // Wrist
    wristMotor = new CANSparkMaxSim(TrapElvConstants.WRIST_MOTOR_ID, MotorType.kBrushed);
    wristMotor.restoreFactoryDefaults();
    wristPIDController =
        new PIDController(
            TrapElvConstants.WRIST_PID[0],
            TrapElvConstants.WRIST_PID[1],
            TrapElvConstants.WRIST_PID[2]);
    wristPIDController.setIZone(TrapElvConstants.WRIST_PID[3]);
    wristFeedforward =
        new ArmFeedforward(
            TrapElvConstants.WRIST_FF[0],
            TrapElvConstants.WRIST_FF[1],
            TrapElvConstants.WRIST_FF[2],
            TrapElvConstants.WRIST_FF[3]);
    TrapElvTab.add("Wrist PID", wristPIDController).withSize(2, 2).withPosition(0, 0);

    wristEncoder = wristMotor.getAbsoluteEncoder();

    rollerMotor = new CANSparkMax(TrapElvConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    rollerMotor.restoreFactoryDefaults();

    sourceBreak = new DigitalInput(TrapElvConstants.SOURCE_BREAK_ID);
    groundBreak = new DigitalInput(TrapElvConstants.GROUND_BREAK_ID);

    offsetEntry = new DebugEntry<Double>(0.0, "Wrist Offset", this);
    currentPositionEntry =
        new DebugEntry<>(wristEncoder.getPosition(), "Current wrist Position", this);

    TrapElvTab.add("Intake Source", intakeSource()).withPosition(2, 0);
    TrapElvTab.add("Intake Ground", intakeGround()).withPosition(3, 0);
    TrapElvTab.add("Stow Wrist", new InstantCommand(() -> stowTrapElv())).withPosition(4, 0);
    TrapElvTab.add("Amp Score", scoreAMP()).withPosition(5, 0);
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
      TrapElvTab.add("Base Elv PID", baseMotor1.getPIDController());

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
      TrapElvTab.add("Scoring Elv PID", scoringMotor.getPIDController());
    }

    // SmartDashboard
    sourceLog = new DebugEntry<Boolean>(sourceBreak.get(), "Source Beam Break", this);
    groundLog = new DebugEntry<Boolean>(groundBreak.get(), "Ground Beam Break", this);

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
              DCMotor.getBag(1),
              TrapElvConstants.WRIST_GEAR_RATIO,
              TrapElvConstants.WRIST_MOI,
              TrapElvConstants.WRIST_LENGTH,
              TrapElvConstants.WRIST_MIN_ANGLE, // min rotation
              TrapElvConstants.WRIST_MAX_ANGLE, // max rotation
              true,
              0);

      TrapElvTab.add("Trap Arm Mech", elvMechanism).withPosition(7, 7);
      wristEncoder.setZeroOffset(TrapElvConstants.WRIST_ZERO_OFFSET);
    }
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

  // Commands
  public Command setRoller(double rollerSpeed) {
    return run(() -> {
          rollerMotor.set(rollerSpeed);
        })
        .withName("Set Roller");
  }

  public Command stopRoller() {
    return run(() -> {
          rollerMotor.stopMotor();
        })
        .withName("Stop Roller");
  }

  public Command intakeSource() {
    return startEnd(
            () -> {
              setTrapArm(TrapElvState.FROM_SOURCE);
              rollerMotor.set(TrapElvConstants.ROLLER_INTAKE_SPEED);
            },
            () -> {
              stowTrapElv();
            })
        .withName("Intake From Source");
  }

  public Command intakeGround() {
    return startEnd(
            () -> {
              setTrapArm(TrapElvState.FROM_INTAKE);
              rollerMotor.set(TrapElvConstants.ROLLER_INTAKE_SPEED);
            },
            () -> {
              stowTrapElv();
            })
        .withName("Intake from Ground");
  }

  public Command scoreAMP() {
    return startEnd(
            () -> {
              setTrapArm(TrapElvState.AMP_SCORE);
              setRoller(-TrapElvConstants.ROLLER_SCORING_SPEED);
            },
            () -> {
              stowTrapElv();
            })
        .withName("Score Amp");
  }

  public Command scoreTrap() {
    return startEnd(
            () -> {
              setTrapArm(TrapElvState.TRAP_SCORE);
              rollerMotor.set(TrapElvConstants.ROLLER_INTAKE_SPEED);
            },
            () -> {
              stowTrapElv();
            })
        .withName("Score Trap");
  }

  public void stowTrapElv() {
    setTrapArm(TrapElvState.STOWED);
    rollerMotor.stopMotor();
  }

  public Command zeroArm() {
    if (isElv) {
      return startEnd(
              () -> {
                // Command for zeroing elevator if elevator happens to be not at zero
                // Runs elevator motors until there limit switches are pressed
                if (!baseLimit.get()) {
                  baseMotor1.set(TrapElvConstants.ELV_ZEROING_SPEED);
                  baseMotor2.set(TrapElvConstants.ELV_ZEROING_SPEED);
                } else {
                  baseMotor1.stopMotor();
                  baseMotor2.stopMotor();
                  baseMotorOffset1 = baseMotor1.getEncoder().getPosition();
                  baseMotorOffset2 = baseMotor2.getEncoder().getPosition();
                }
                if (!scoringLimit.get()) {
                  scoringMotor.set(TrapElvConstants.ELV_ZEROING_SPEED);
                } else {
                  scoringMotor.stopMotor();
                  scoringMotorOffset = scoringMotor.getEncoder().getPosition();
                }
              },
              () -> {
                baseMotor1.stopMotor();
                baseMotor2.stopMotor();
                scoringMotor.stopMotor();
              })
          .withName("Zero Arm");
    } else {
      return run(() -> {});
    }
  }

  public void setTrapArm(TrapElvState state) {
    wristState = state.getWristPose();
    wristGoal.setDouble(wristState);
    if (isElv) {
      baseGoal.setDouble(Units.metersToInches(TrapElvConstants.ELV_MIN_HEIGHT) + state.basePose);
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
    currentPositionEntry.log(wristEncoder.getPosition());
    sourceLog.log(sourceBreak.get());
    groundLog.log(groundBreak.get());
    if (isElv) {
      baseLog.log(baseLimit.get());
      scoringLog.log(sourceBreak.get());
    }

    FF = wristFeedforward.calculate(wristEncoder.getPosition(), 0);
    wristMotor.setVoltage(
        MathUtil.clamp(
            wristPIDController.calculate(
                    Units.rotationsToDegrees(wristEncoder.getPosition()),
                    Units.rotationsToDegrees(wristState))
                + FF,
            -12,
            12));
    FFOutput.setDouble(FF);

    wristOutput.setDouble(wristMotor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
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
    }

    // Wrist Sim Stuff
    m_wristMotorSim.setInput(wristMotor.getAppliedOutput());
    m_wristMotorSim.update(Robot.defaultPeriodSecs);
    wristMotor.setAbsolutePosition(Units.radiansToRotations(m_wristMotorSim.getAngleRads()));
    // Offest added so that gravity is simulated in the right direction
    wristMech.setAngle(Units.radiansToDegrees(m_wristMotorSim.getAngleRads()) - 90);
    SmartDashboard.putNumber("Current Draw Wrist (A)", m_wristMotorSim.getCurrentDrawAmps());
    SmartDashboard.putNumber(
        "Wrist Sim Angle", Units.radiansToRotations(m_wristMotorSim.getAngleRads()));

    if (isElv) {
      SmartDashboard.putNumber("base CAN Sim", baseMotor1.get());
      SmartDashboard.putNumber("scoring CAN Sim", scoringMotor.get());

      baseMech.setLength(m_baseElevatorSim.getPositionMeters());
      scoringMech.setLength(m_scoringElevatorSim.getPositionMeters());

      SmartDashboard.putNumber(
          "Base Elv Length", Units.metersToInches(m_baseElevatorSim.getPositionMeters()));
      SmartDashboard.putNumber(
          "Scoring Elv Length", Units.metersToInches(m_scoringElevatorSim.getPositionMeters()));
    }
  }
}

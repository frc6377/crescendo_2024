// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxSim;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapElvConstants;
import frc.robot.Robot;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.TOFSensorSimple;
import java.util.function.BooleanSupplier;

public class TrapElvSubsystem extends SubsystemBase {
  private final boolean isElv = false;

  // Wrist motors
  private final CANSparkMax wristMotor;
  private double wristState;
  private final PIDController wristPIDController;

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
  private final TOFSensorSimple groundBreak;

  // Limit Switches
  private DigitalInput baseLimit;
  private DigitalInput scoringLimit;

  // Encoders
  private final CANcoder wristEncoder;
  private SimDeviceSim simWristCoder;
  private SimDouble simWristPos;

  private DebugEntry<Boolean> sourceLog;
  private DebugEntry<Boolean> groundLog;
  private DebugEntry<Boolean> baseLog;
  private DebugEntry<Boolean> scoringLog;

  private Mechanism2d elvMechanism;
  private MechanismRoot2d root;
  private MechanismLigament2d baseMech;
  private MechanismLigament2d scoringMech;
  private MechanismLigament2d wristMech;
  private Mechanism2d rollerMech;
  private MechanismRoot2d rollerRoot2d;
  private MechanismLigament2d rollerMechLigmt;

  private ElevatorSim m_baseElevatorSim;
  private ElevatorSim m_scoringElevatorSim;
  private SingleJointedArmSim m_wristMotorSim;

  private ShuffleboardTab TrapElvTab = Shuffleboard.getTab("Trap Arm Tab");
  private GenericEntry baseGoal = TrapElvTab.add("Base Goal", 0).getEntry();
  private GenericEntry sourceBreakDis = TrapElvTab.add("Source Break Distance", 0).getEntry();
  private GenericEntry groundBreakDis = TrapElvTab.add("Ground Break Distance", 0).getEntry();

  // States
  public static enum TrapElvState {
    // Degrees, elv height, elv height
    STOWED(-0.25, 0.0, 0.0),
    FROM_INTAKE(-0.25, 0.0, 0.0),
    FROM_SOURCE(-0.1, 0.0, 12.0),
    TRAP_SCORE(0.0, 12.0, 12.0),
    AMP_SCORE(0.7, 0.0, 12.0);

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
    wristMotor = new CANSparkMax(TrapElvConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    wristPIDController =
        new PIDController(
            TrapElvConstants.WRIST_PID[0],
            TrapElvConstants.WRIST_PID[1],
            TrapElvConstants.WRIST_PID[2]);
    TrapElvTab.add("Wrist PID", wristPIDController);

    wristEncoder = new CANcoder(6);

    rollerMotor = new CANSparkMax(TrapElvConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    rollerMotor.restoreFactoryDefaults();

    sourceBreak =
        new TOFSensorSimple(TrapElvConstants.SOURCE_BREAK_ID, TrapElvConstants.WRIST_BREAK_THOLD);
    groundBreak =
        new TOFSensorSimple(TrapElvConstants.GROUND_BREAK_ID, TrapElvConstants.WRIST_BREAK_THOLD);

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
    sourceLog = new DebugEntry<Boolean>(sourceBreak.isBeamBroke(), "Source Beam Break", this);
    sourceBreakDis.setDouble(sourceBreak.getMilliMeters());
    groundLog = new DebugEntry<Boolean>(groundBreak.isBeamBroke(), "Ground Beam Break", this);
    groundBreakDis.setDouble(groundBreak.getMilliMeters());

    // Simulation
    if (Robot.isSimulation()) {
      // 2D Mechanism Wrist
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

      // Roller Mech2D
      rollerMech = new Mechanism2d(2, 2);
      rollerRoot2d = rollerMech.getRoot("Root", 1, 1);
      rollerMechLigmt =
          rollerRoot2d.append(
              new MechanismLigament2d("Roller Ligmt", 10, 0, 20, new Color8Bit(Color.kYellow)));

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
              (1.0 / 3.0)
                  * TrapElvConstants.ELV_LIFT_MASS
                  * Math.pow(TrapElvConstants.WRIST_LENGTH, 2.0),
              TrapElvConstants.WRIST_LENGTH,
              TrapElvConstants.WRIST_MIN_ANGLE, // min rotation
              TrapElvConstants.WRIST_MAX_ANGLE, // max rotation
              true,
              0);

      TrapElvTab.add("Trap Arm Mech", elvMechanism);
      TrapElvTab.add("Roller Mech", rollerMech);

      simWristCoder = new SimDeviceSim("CANEncoder:CANCoder (v6)", wristEncoder.getDeviceID());
      simWristPos = simWristCoder.getDouble("rawPositionInput");
      wristEncoder.setPosition(0);
    }
  }

  // Boolean Suppliers
  public BooleanSupplier getSourceBreakBool() {
    return () -> sourceBreak.isBeamBroke();
  }

  public BooleanSupplier getSourceBreakBoolInverse() {
    return () -> !sourceBreak.isBeamBroke();
  }

  public BooleanSupplier getGroundBreakBool() {
    return () -> groundBreak.isBeamBroke();
  }

  public BooleanSupplier getGroundBreakBoolInverse() {
    return () -> !groundBreak.isBeamBroke();
  }

  public TOFSensorSimple getSourceBreak() {
    return sourceBreak;
  }

  public BooleanSupplier getBaseLimit() {
    return () -> baseLimit.get();
  }

  public BooleanSupplier getScoringLimit() {
    return () -> scoringLimit.get();
  }

  // Commands
  public Command setRoller(double s) {
    return run(() -> {
          rollerMotor.set(s);
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
              rollerMotor.set(-TrapElvConstants.ROLLER_INTAKE_SPEED);
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
    SmartDashboard.putNumber("Wrist Goal", state.wristPose);
    SmartDashboard.putNumber(
        "Wrist PID Control Output",
        wristPIDController.calculate(
            wristEncoder.getPosition().getValueAsDouble(), state.getWristPose()));
    wristState = state.getWristPose();
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
    sourceLog.log(sourceBreak.isBeamBroke());
    groundLog.log(groundBreak.isBeamBroke());
    sourceBreakDis.setDouble(sourceBreak.getMilliMeters());
    groundBreakDis.setDouble(groundBreak.getMilliMeters());

    if (isElv) {
      baseLog.log(baseLimit.get());
      scoringLog.log(scoringLimit.get());
    }
    wristMotor.set(
        MathUtil.clamp(
            wristPIDController.calculate(wristEncoder.getPosition().getValueAsDouble(), wristState),
            -1,
            1));
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
    m_wristMotorSim.setInput(wristMotor.get() * RobotController.getBatteryVoltage());
    m_wristMotorSim.update(Robot.defaultPeriodSecs);
    simWristPos.set(Units.radiansToRotations(m_wristMotorSim.getAngleRads()));
    // Offest added so that gravity is simulated in the right direction
    wristMech.setAngle(Units.radiansToDegrees(m_wristMotorSim.getAngleRads()) - 90);

    SmartDashboard.putNumber("Wrist Motor Output", wristMotor.get());
    SmartDashboard.putNumber(
        "Wrist Sim Angle", Units.radiansToRotations(m_wristMotorSim.getAngleRads()));

    rollerMechLigmt.setAngle(rollerMechLigmt.getAngle() + (rollerMotor.get() * 12));

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

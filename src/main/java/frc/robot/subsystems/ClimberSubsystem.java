package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMaxSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Robot;
import frc.robot.utilities.DebugEntry;
import frc.robot.utilities.TunableNumber;

public class ClimberSubsystem extends SubsystemBase {
  final CANSparkMaxSim leftMotorController;
  final CANSparkMaxSim rightMotorController;
  final SparkPIDController leftPidController;
  final SparkPIDController rightPidController;
  final RelativeEncoder leftEncoder;
  final RelativeEncoder rightEncoder;

  private DebugEntry<Double> leftClimbPosition;
  private DebugEntry<Double> rightClimbPosition;
  private DebugEntry<String> goalName;
  private DebugEntry<Double> goalPosition;
  private DebugEntry<Double> motorGoalEntry;
  private DebugEntry<Double> leftMotorOutput;
  private DebugEntry<Double> rightMotorOutput;

  private SingleJointedArmSim leftClimberArmSim;
  private SingleJointedArmSim rightClimberArmSim;

  private Mechanism2d leftClimberMech;
  private MechanismRoot2d leftMechRoot;
  private MechanismLigament2d leftMechBase;
  private MechanismLigament2d leftMechArm;
  private Mechanism2d rightClimberMech;
  private MechanismRoot2d rightMechRoot;
  private MechanismLigament2d rightMechBase;
  private MechanismLigament2d rightMechArm;

  public enum climbStates {
    PICK_UP(0.25),
    LIFT_SETPOINT(-0.1),
    IDLE_SETPOINT(-0.25);

    private double angle;

    climbStates(double angle) {
      this.angle = angle;
    }

    public double getMotorAngle() {
      return angle * ClimberConstants.GEAR_RATIO;
    }

    public double getStateAngle() {
      return angle;
    }
  }

  private ShuffleboardTab ClimberTab = Shuffleboard.getTab("ClimberSubsystem");
  private TunableNumber P;
  private TunableNumber I;
  private TunableNumber D;

  public ClimberSubsystem() {
    leftMotorController = new CANSparkMaxSim(ClimberConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotorController =
        new CANSparkMaxSim(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftMotorController.restoreFactoryDefaults();
    rightMotorController.restoreFactoryDefaults();

    // leftMotorController.setInverted(ClimberConstants.LEFT_IS_INVERTED);
    // rightMotorController.setInverted(ClimberConstants.RIGHT_IS_INVERTED);

    leftPidController = leftMotorController.getPIDController();
    rightPidController = rightMotorController.getPIDController();

    leftEncoder = leftMotorController.getEncoder();
    rightEncoder = rightMotorController.getEncoder();

    leftPidController.setP(ClimberConstants.POSITION_P_GAIN);
    leftPidController.setI(ClimberConstants.POSITION_I_GAIN);
    leftPidController.setD(ClimberConstants.POSITION_D_GAIN);
    rightPidController.setP(ClimberConstants.POSITION_P_GAIN);
    rightPidController.setI(ClimberConstants.POSITION_I_GAIN);
    rightPidController.setD(ClimberConstants.POSITION_D_GAIN);
    P =
        new TunableNumber(
            "P",
            ClimberConstants.POSITION_P_GAIN,
            P -> {
              leftPidController.setP(P);
              rightPidController.setP(P);
            });
    I =
        new TunableNumber(
            "P",
            ClimberConstants.POSITION_I_GAIN,
            I -> {
              leftPidController.setI(I);
              rightPidController.setI(I);
            });
    D =
        new TunableNumber(
            "P",
            ClimberConstants.POSITION_D_GAIN,
            D -> {
              leftPidController.setD(D);
              rightPidController.setD(D);
            });

    if (Robot.isSimulation()) {
      leftClimberArmSim =
          new SingleJointedArmSim(
              ClimberConstants.CLIMBER_MECH_MOTORS,
              ClimberConstants.GEAR_RATIO, // double gearing0,
              ClimberConstants.CLIMBER_MOI, // double jKgMetersSquared,
              ClimberConstants.CLIMBER_LENGTH, // double armLengthMeters,
              ClimberConstants.CLIMBER_MIN_ANGLE, // double minAngleRads,
              ClimberConstants.CLIMBER_MAX_ANGLE, // double maxAngleRads,
              true, // boolean simulateGravity,
              0 // double startingAngleRads
              );

      rightClimberArmSim =
          new SingleJointedArmSim(
              ClimberConstants.CLIMBER_MECH_MOTORS,
              ClimberConstants.GEAR_RATIO, // double gearing0,
              ClimberConstants.CLIMBER_MOI, // double jKgMetersSquared,
              ClimberConstants.CLIMBER_LENGTH, // double armLengthMeters,
              ClimberConstants.CLIMBER_MIN_ANGLE, // double minAngleRads,
              ClimberConstants.CLIMBER_MAX_ANGLE, // double maxAngleRads,
              true, // boolean simulateGravity,
              0 // double startingAngleRads
              );

      leftClimberMech = new Mechanism2d(2, 2);
      leftMechRoot = leftClimberMech.getRoot("Climber Left Arm Root", 1, 0);
      leftMechBase =
          leftMechRoot.append(
              new MechanismLigament2d(
                  "Climber Left Base Lig", ClimberConstants.CLIMBER_BASE_LENGTH, 90));
      leftMechArm =
          leftMechBase.append(
              new MechanismLigament2d(
                  "Climber Left Arm Lig",
                  ClimberConstants.CLIMBER_LENGTH,
                  leftClimberArmSim.getAngleRads()));

      rightClimberMech = new Mechanism2d(2, 2);
      rightMechRoot = rightClimberMech.getRoot("Climber Right Arm Root", 1, 0);
      rightMechBase =
          rightMechRoot.append(
              new MechanismLigament2d(
                  "Climber Right Base Lig", ClimberConstants.CLIMBER_BASE_LENGTH, 90));
      rightMechArm =
          rightMechBase.append(
              new MechanismLigament2d(
                  "Climber Right Arm Lig",
                  ClimberConstants.CLIMBER_LENGTH,
                  rightClimberArmSim.getAngleRads()));

      ClimberTab.add("Left Climber Mech", leftClimberMech);
      ClimberTab.add("Right Climber Mech", rightClimberMech);
    }

    leftClimbPosition =
        new DebugEntry<Double>(leftEncoder.getPosition(), "Current right climb position", this);
    rightClimbPosition =
        new DebugEntry<Double>(rightEncoder.getPosition(), "Current left climb position", this);
    goalPosition =
        new DebugEntry<Double>(
            climbStates.IDLE_SETPOINT.getStateAngle(), "Current Climber Position Goal", this);
    goalName =
        new DebugEntry<String>(climbStates.IDLE_SETPOINT.name(), "Current Climber State", this);
    motorGoalEntry =
        new DebugEntry<Double>(
            climbStates.IDLE_SETPOINT.getMotorAngle(), "Current Motor Goal", this);
    leftMotorOutput =
        new DebugEntry<Double>(leftMotorController.get(), "Left Climber Output", this);
    rightMotorOutput =
        new DebugEntry<Double>(rightMotorController.get(), "Right Climber Output", this);
  }

  public void applyDemand() {
    leftMotorController.setVoltage(ClimberConstants.CLIP_VOLTAGE);
    rightMotorController.setVoltage(ClimberConstants.CLIP_VOLTAGE);
  }

  public void stopMotors() {
    leftMotorController.setVoltage(0);
    rightMotorController.setVoltage(0);
  }

  public Command climbLowerCommand() {
    return startEnd(
            () -> {
              applyDemand();
            },
            () -> {
              stopMotors();
            })
        .withName("climbLowerCommand");
  }

  public void gotoPositionCommand(climbStates state) {
    goalName.log(state.name());
    goalPosition.log(state.getStateAngle());
    motorGoalEntry.log(state.getMotorAngle());
    if (Robot.isSimulation()) {
      if (state.equals(climbStates.LIFT_SETPOINT)) {
        leftMotorController.set(1);
        rightMotorController.set(1);
      } else if (state.equals(climbStates.PICK_UP)) {
        leftMotorController.set(-1);
        rightMotorController.set(-1);
      } else {
        leftMotorController.set(0);
        rightMotorController.set(0);
      }
    } else {
      leftPidController.setReference(state.getMotorAngle(), ControlType.kPosition);
      rightPidController.setReference(state.getMotorAngle(), ControlType.kPosition);
    }
  }

  public Command gotoPositionCommand(climbStates state, double x) {
    goalName.log(state.name());
    goalPosition.log(state.getStateAngle());
    return runEnd(() -> {}, () -> {});
  }

  public Command gotoLiftPositionCommand() {
    return runEnd(
            () -> {
              gotoPositionCommand(climbStates.LIFT_SETPOINT);
            },
            () -> {
              stopMotors();
            })
        .withName("Lift Pose Command");
  }

  public Command gotoRaisePositionCommand() {
    return runEnd(
            () -> {
              gotoPositionCommand(climbStates.PICK_UP);
            },
            () -> {
              stopMotors();
            })
        .withName("Raise Pose Command");
  }

  public Command gotoIdlePositionCommand() {
    return runEnd(
            () -> {
              gotoPositionCommand(climbStates.IDLE_SETPOINT);
            },
            () -> {
              stopMotors();
            })
        .withName("Idle Pose Command");
  }

  @Override
  public void periodic() {
    rightClimbPosition.log(rightEncoder.getPosition());
    leftClimbPosition.log(leftEncoder.getPosition());

    leftMotorOutput.log(leftMotorController.get());
    rightMotorOutput.log(rightMotorController.get());

    double P_Value = P.get();
    double I_Value = I.get();
    double D_Value = D.get();
    if (P_Value != leftMotorController.getPIDController().getP()) {
      leftMotorController.getPIDController().setP(P_Value);
    }
    if (I_Value != leftMotorController.getPIDController().getI()) {
      leftMotorController.getPIDController().setI(I_Value);
    }
    if (D_Value != leftMotorController.getPIDController().getD()) {
      leftMotorController.getPIDController().setD(D_Value);
    }
  }

  @Override
  public void simulationPeriodic() {
    for (double i = 0; i < Robot.defaultPeriodSecs; i += CANSparkMaxSim.kPeriod) {
      // Wrist Sim Stuff
      leftClimberArmSim.setInput(leftMotorController.getAppliedOutput());
      leftClimberArmSim.update(CANSparkMaxSim.kPeriod);
      leftEncoder.setPosition(Units.radiansToRotations(leftClimberArmSim.getAngleRads()));
      // Offest added so that gravity is simulated in the right direction
      leftMechArm.setAngle(Units.radiansToDegrees(leftClimberArmSim.getAngleRads()) - 90);
      SmartDashboard.putNumber("Current Draw Wrist (A)", leftClimberArmSim.getCurrentDrawAmps());

      // Wrist Sim Stuff
      rightClimberArmSim.setInput(rightMotorController.getAppliedOutput());
      rightClimberArmSim.update(CANSparkMaxSim.kPeriod);
      rightEncoder.setPosition(Units.radiansToRotations(rightClimberArmSim.getAngleRads()));
      // Offest added so that gravity is simulated in the right direction
      rightMechArm.setAngle(Units.radiansToDegrees(rightClimberArmSim.getAngleRads()) - 90);
      SmartDashboard.putNumber("Current Draw Wrist (A)", rightClimberArmSim.getCurrentDrawAmps());
    }
  }
}

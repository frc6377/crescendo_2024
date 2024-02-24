package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Robot;
import frc.robot.utilities.DebugEntry;
import java.util.function.BooleanSupplier;

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

    public double getStateAngle() {
      return angle;
    }
  }

  private ShuffleboardTab ClimberTab = Shuffleboard.getTab("Climber Tab");
  double target = 0;

  public ClimberSubsystem() {
    leftMotorController = new CANSparkMaxSim(ClimberConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotorController = new CANSparkMaxSim(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

    leftMotorController.restoreFactoryDefaults();
    rightMotorController.restoreFactoryDefaults();

    leftMotorController.setInverted(ClimberConstants.LEFT_IS_INVERTED);
    rightMotorController.setInverted(ClimberConstants.RIGHT_IS_INVERTED);

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
                  "Climber Left Base Lig", ClimberConstants.CLIMBER_BASE_LENGTH, 0));
      leftMechArm =
          leftMechBase.append(
              new MechanismLigament2d(
                  "Climber Left Arm Lig",
                  ClimberConstants.CLIMBER_LENGTH,
                  leftClimberArmSim.getAngleRads()));

      rightClimberMech = new Mechanism2d(2, 2);
      rightMechRoot = rightClimberMech.getRoot("Climber Right Arm Root", 1, 0);
      rightMechBase = rightMechRoot.append(new MechanismLigament2d("Climber Right Base Lig", ClimberConstants.CLIMBER_BASE_LENGTH, 0));
      rightMechArm = rightMechBase.append(new MechanismLigament2d("Climber Right Arm Lig", ClimberConstants.CLIMBER_BASE_LENGTH, rightClimberArmSim.getAngleRads()));

      ClimberTab.add(leftClimberMech);
      ClimberTab.add(rightClimberMech);
    }

    ClimberTab.add("Left Climb PID", leftPidController).withSize(2, 2).withPosition(0, 0);
    ClimberTab.add("Right Climb PID", rightPidController).withSize(2, 2).withPosition(2, 0);
    leftClimbPosition =
        new DebugEntry<Double>(leftEncoder.getPosition(), "Current right climb position", this);
    rightClimbPosition =
        new DebugEntry<Double>(rightEncoder.getPosition(), "Current left climb position", this);
    goalPosition = new DebugEntry<Double>(climbStates.IDLE_SETPOINT.getStateAngle(),"Current Goal Position", this);
    goalName = new DebugEntry<String>(climbStates.IDLE_SETPOINT.name(), "Current State Name", this);
    leftMotorOutput = new DebugEntry<Double>(leftMotorController.get(), "Left Motor Output", this);
    rightMotorOutput = new DebugEntry<Double>(rightMotorController.get(), "Right Motor Output", this);
  }

  public void applyDemand() {
    leftMotorController.setVoltage(ClimberConstants.CLIP_VOLTAGE);
    rightMotorController.setVoltage(ClimberConstants.CLIP_VOLTAGE);
  }

  public void stopMotors(){
    leftMotorController.stopMotor();
    rightMotorController.stopMotor();
  }

  public void gotoPosition(double rotation) {
    target = rotation * Constants.ClimberConstants.GEAR_RATIO;
    leftPidController.setReference(target, ControlType.kPosition);
    rightPidController.setReference(target, ControlType.kPosition);
  }

  public Command gotoPositionCommand(climbStates state) {
    Runnable init = () -> gotoPosition(state.getStateAngle());
    BooleanSupplier done =
        () ->
            Math.abs(rightEncoder.getPosition() - target) < Constants.ClimberConstants.MAX_ERROR
                && Math.abs(leftEncoder.getPosition() - target)
                    < Constants.ClimberConstants.MAX_ERROR;
    return new FunctionalCommand(init, () -> {}, (interupt) -> {}, done, this);
  }

  public Command gotoLiftPositionCommand() {
    return gotoPositionCommand(climbStates.PICK_UP);
  }

  public Command gotoRaisePositionCommand() {
    return gotoPositionCommand(climbStates.LIFT_SETPOINT);
  }

  public Command gotoIdlePositionCommand() {
    return gotoPositionCommand(climbStates.IDLE_SETPOINT);
  }

  @Override
  public void periodic() {
    rightClimbPosition.log(rightEncoder.getPosition());
    leftClimbPosition.log(leftEncoder.getPosition());

    leftMotorOutput.log(leftMotorController.get());
    rightMotorOutput.log(rightMotorController.get());
  }

  @Override
  public void simulationPeriodic() {
    for (double i = 0; i < Robot.defaultPeriodSecs; i += CANSparkMaxSim.kPeriod) {
      // Wrist Sim Stuff
      leftClimberArmSim.setInput(leftMotorController.getAppliedOutput());
      leftClimberArmSim.update(CANSparkMaxSim.kPeriod);
      leftMotorController.setAbsolutePosition(Units.radiansToRotations(leftClimberArmSim.getAngleRads()));
      // Offest added so that gravity is simulated in the right direction
      leftMechArm.setAngle(Units.radiansToDegrees(leftClimberArmSim.getAngleRads()) - 90);
      SmartDashboard.putNumber("Current Draw Wrist (A)", leftClimberArmSim.getCurrentDrawAmps());

      // Wrist Sim Stuff
      rightClimberArmSim.setInput(rightMotorController.getAppliedOutput());
      rightClimberArmSim.update(CANSparkMaxSim.kPeriod);
      rightMotorController.setAbsolutePosition(Units.radiansToRotations(rightClimberArmSim.getAngleRads()));
      // Offest added so that gravity is simulated in the right direction
      rightMechArm.setAngle(Units.radiansToDegrees(rightClimberArmSim.getAngleRads()) - 90);
      SmartDashboard.putNumber("Current Draw Wrist (A)", rightClimberArmSim.getCurrentDrawAmps());
    }
  }
}

package frc.robot.subsystems.climberSubsystem;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utilities.DebugEntry;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax leftArmMotor;
  private final CANSparkMax rightArmMotor;

  private PIDState pidState;
  private final Timer timer = new Timer();
  private double nextActionTime = 1;
  private ShuffleboardTab climberTab = Shuffleboard.getTab(this.getName());

  private ClimberState state = ClimberState.DEFAULT;

  private final Servo leftServo;
  private final Servo rightServo;

  private final DebugEntry<Double> rightArmPoseEntry;
  private DebugEntry<Double> leftArmPoseEntry;
  private DebugEntry<Double> rightArmOutputEntry;
  private DebugEntry<Double> leftArmOutputEntry;
  private DebugEntry<String> currentCommand;

  public ClimberSubsystem() {
    leftServo = new Servo(Constants.ClimberConstants.LEFT_SERVO_PORT);
    rightServo = new Servo(Constants.ClimberConstants.RIGHT_SERVO_PORT);

    leftArmMotor = new CANSparkMax(ClimberConstants.LEFT_ARM_ID, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(ClimberConstants.RIGHT_ARM_ID, MotorType.kBrushless);
    configMotor(leftArmMotor, true);
    configMotor(rightArmMotor, false);

    rightArmPoseEntry =
        new DebugEntry<Double>(
            rightArmMotor.getEncoder().getPosition(), "right arm position", this);
    leftArmPoseEntry =
        new DebugEntry<Double>(rightArmMotor.getEncoder().getPosition(), "left arm position", this);
    rightArmOutputEntry =
        new DebugEntry<Double>(rightArmMotor.getAppliedOutput(), "right arm output", this);
    leftArmOutputEntry =
        new DebugEntry<Double>(rightArmMotor.getAppliedOutput(), "left arm output", this);
    currentCommand = new DebugEntry<String>("none", "Climber Command", this);
  }

  private void configMotor(CANSparkMax motor, boolean invert) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    SparkPIDController pidController = motor.getPIDController();
    pidController.setP(ClimberConstants.CURRENT_PID[0]);
    pidController.setI(ClimberConstants.CURRENT_PID[1]);
    pidController.setD(ClimberConstants.CURRENT_PID[2]);
    double armPosition;

    armPosition = motor.getAbsoluteEncoder().getPosition();
    if (armPosition > 0.5) armPosition -= 1;
    motor.setSoftLimit(
        SoftLimitDirection.kForward,
        (float) (Units.degreesToRotations(135) * ClimberConstants.GEAR_RATIO));
    motor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    double motorPosition = armPosition * ClimberConstants.GEAR_RATIO;
    motor.getEncoder().setPosition(motorPosition);
    motor.setInverted(invert);
    motor.setSmartCurrentLimit(40);
  }

  /**
   * Positive is up, negative is down. Is even between arms.
   *
   * @param speed the speed to set (-1 to 1)
   */
  public void applyPercent(double speed) {
    leftArmMotor.set(speed);
    rightArmMotor.set(speed);
  }

  /**
   * Positive is up, negative is down. Is even between arms.
   *
   * @param current the current to set
   */
  public void applyCurrentDemand(double current) {
    leftArmMotor.getPIDController().setIAccum(0);
    rightArmMotor.getPIDController().setIAccum(0);
    leftArmMotor.getPIDController().setReference(current, ControlType.kCurrent);
    rightArmMotor.getPIDController().setReference(current, ControlType.kCurrent);
  }

  /**
   * Any command that uses this needs to clean up after self. As in reset min and max to -1 and 1
   *
   * @param max the maximum output (Physically 1)
   * @param min the minimum output (Physically -1)
   */
  public void setOutputLimits(double max, double min) {
    leftArmMotor.getPIDController().setOutputRange(min, max);
    rightArmMotor.getPIDController().setOutputRange(min, max);
  }

  public Position getPosition() {
    return new Position(
        leftArmMotor.getEncoder().getPosition(), rightArmMotor.getEncoder().getPosition());
  }

  public CurrentVelocity getVelocity() {
    return new CurrentVelocity(
        leftArmMotor.getEncoder().getVelocity(), rightArmMotor.getEncoder().getVelocity());
  }

  protected ClimberState getCurrentState() {
    return state;
  }

  protected void setState(ClimberState pos) {
    this.state = pos;
    updateMotors();
  }

  protected void advanceState() {
    switch (state) {
      case MAXIMUM:
        setState(ClimberState.CLIP);
        break;
      case CLIP:
        setState(ClimberState.CLIMB);
        break;
    }
  }

  private void updateMotors() {
    switch (state) {
      case INITIAL_RAISE:
        applyPercent(ClimberConstants.INITAL_RAISE_PERCENT);
        setServoPosition(Constants.ClimberConstants.SERVO_OFF_POSITION);
        timer.reset();
        timer.start();
        break;
      case MAXIMUM:
        setServoPosition(Constants.ClimberConstants.SERVO_OFF_POSITION);
        setOutputLimits(1, 0);
        applyCurrentDemand(ClimberConstants.RAISE_CURRENT);
        break;
      case CLIP:
        setServoPosition(Constants.ClimberConstants.SERVO_OFF_POSITION);
        applyCurrentDemand(ClimberConstants.CLIP_CURRENT);
        break;
      case CLIMB:
        setServoPosition(Constants.ClimberConstants.SERVO_ON_POSITION);
        requestPosition(ClimberConstants.CLIMB_POSITION);
        break;
      default:
        break;
    }
  }

  /**
   * Sets the position of the Servo motors
   *
   * @param pos The position of the motors in rotations
   */
  protected void setServoPosition(double pos) {
    leftServo.set(pos);
    rightServo.set(pos);
  }

  /**
   * Sets the position of the climber motors
   *
   * @param climbPosition The position of the motors in degrees
   */
  public void requestPosition(double climbPosition) {
    setPIDState(PIDState.POSITION);
    leftArmMotor.getPIDController().setIAccum(0);
    rightArmMotor.getPIDController().setIAccum(0);
    leftArmMotor.getPIDController().setReference(climbPosition, ControlType.kPosition);
    rightArmMotor.getPIDController().setReference(climbPosition, ControlType.kPosition);
  }

  private void setPIDState(PIDState state) {
    if (state == pidState) return;
    configPID(rightArmMotor.getPIDController(), state.getPID());
    configPID(leftArmMotor.getPIDController(), state.getPID());
  }

  private static void configPID(SparkPIDController pidController, double[] pid) {
    if (pid.length != 4) throw new RuntimeException("Incorrect number of PID numbers");
    pidController.setP(pid[0]);
    pidController.setI(pid[1]);
    pidController.setD(pid[2]);
    pidController.setFF(pid[3]);
  }

  @Override
  public void periodic() {
    rightArmPoseEntry.log(rightArmMotor.getEncoder().getPosition());
    leftArmPoseEntry.log(leftArmMotor.getEncoder().getPosition());
    rightArmOutputEntry.log(rightArmMotor.get());
    leftArmOutputEntry.log(leftArmMotor.get());
    if (this.getCurrentCommand() != null) currentCommand.log(this.getCurrentCommand().getName());

    if (timer.hasElapsed(nextActionTime)) {
      switch (state) {
        case INITIAL_RAISE:
          setState(ClimberState.MAXIMUM);
          timer.stop();
          timer.reset();
          break;
        case MAXIMUM:
          if (getVelocity().isZero(0.5)) {
            setOutputLimits(0, -1);
            applyPercent(0);
            timer.stop();
            timer.reset();
          }
          break;
      }
    }
  }

  public record DifferentialDemand(double left, double right) {}

  public record Position(double left, double right) {

    public boolean isLessThen(double arg) {
      return left < arg && right < arg;
    }
  }

  public record AppliedCurrent(double left, double right) {
    public boolean greater(double arg) {

      return left > arg && right > arg;
    }
  }

  public record CurrentVelocity(double left, double right) {
    public boolean isZero(double epsilion) {
      return Math.abs(left) < epsilion && Math.abs(right) < epsilion;
    }
  }

  protected enum ClimberState {
    DEFAULT,
    INITIAL_RAISE,
    MAXIMUM,
    CLIP,
    CLIMB;
  }

  private enum PIDState {
    CURRENT(0),
    POSITION(1);
    private int id;

    private PIDState(int id) {
      this.id = id;
    }

    public double[] getPID() {
      switch (this.id) {
        case 0:
          return ClimberConstants.CURRENT_PID;
        case 1:
          return ClimberConstants.POSITION_PID;
        default:
          return new double[] {0, 0, 0, 0};
      }
    }
  }
}

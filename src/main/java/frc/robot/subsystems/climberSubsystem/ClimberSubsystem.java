package frc.robot.subsystems.climberSubsystem;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private PIDState pidState;
  private CANSparkMax leftArmMotor;
  private CANSparkMax rightArmMotor;
  private ShuffleboardTab climberTab = Shuffleboard.getTab(this.getName());

  public ClimberSubsystem() {
    leftArmMotor = new CANSparkMax(ClimberConstants.LEFT_ARM_ID, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(ClimberConstants.RIGHT_ARM_ID, MotorType.kBrushless);
    configMotor(leftArmMotor);
    configMotor(rightArmMotor);
    leftArmMotor.setInverted(true);

    climberTab.addDouble("right arm position", () -> rightArmMotor.getEncoder().getPosition());
    climberTab.addDouble("left arm position", () -> leftArmMotor.getEncoder().getPosition());
    climberTab.addDouble("right arm output", () -> rightArmMotor.getAppliedOutput());
    climberTab.addDouble("left arm  output", () -> leftArmMotor.getAppliedOutput());
  }

  private void configMotor(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);
    SparkPIDController pidController = motor.getPIDController();
    pidController.setP(ClimberConstants.CURRENT_PID[0]);
    pidController.setI(ClimberConstants.CURRENT_PID[1]);
    pidController.setD(ClimberConstants.CURRENT_PID[2]);
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
   * @param speed the speed to set (-1 to 1)
   */
  public void applyDiffertialPercent(DifferentialDemand demand) {
    leftArmMotor.set(demand.left);
    rightArmMotor.set(demand.right);
  }

  /**
   * Positive is up, negative is down. Is even between arms.
   *
   * @param current the current to set
   */
  public void applyCurrentDemand(double current) {
    leftArmMotor.getPIDController().setReference(current, ControlType.kCurrent);
    rightArmMotor.getPIDController().setReference(current, ControlType.kCurrent);
  }

  /**
   * Apply a currentlimit
   *
   * @param currentLimit the limit in amps to set
   */
  public void setCurrentLimit(int currentLimit) {
    leftArmMotor.setSmartCurrentLimit(currentLimit);
    rightArmMotor.setSmartCurrentLimit(currentLimit);
  }

  public void zeroAtCurrentPosition() {
    leftArmMotor.getEncoder().setPosition(0);
    leftArmMotor.getEncoder().setPosition(0);
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

  public AppliedCurrent getCurrent() {
    return new AppliedCurrent(leftArmMotor.getOutputCurrent(), rightArmMotor.getOutputCurrent());
  }

  public void requestPosition(double climbPosition) {
    setPIDState(PIDState.POSITION);
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
  public void periodic() {}

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

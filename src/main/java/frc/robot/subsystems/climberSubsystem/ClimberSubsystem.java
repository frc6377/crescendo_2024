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
  private CANSparkMax leftArmMotor;
  private CANSparkMax rightArmMotor;
  private ShuffleboardTab climberTab = Shuffleboard.getTab(this.getName());

  public ClimberSubsystem() {
    leftArmMotor = new CANSparkMax(ClimberConstants.LEFT_ARM_ID, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(ClimberConstants.RIGHT_ARM_ID, MotorType.kBrushless);
    configMotor(leftArmMotor);
    configMotor(rightArmMotor);

    climberTab.addDouble("right arm position", () -> rightArmMotor.getEncoder().getPosition());
    climberTab.addDouble("left arm position", () -> leftArmMotor.getEncoder().getPosition());
    climberTab.addDouble("right arm output", () -> rightArmMotor.getAppliedOutput());
    climberTab.addDouble("left arm  output", () -> leftArmMotor.getAppliedOutput());
  }

  private void configMotor(CANSparkMax motor) {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kCoast);
    SparkPIDController pidController = motor.getPIDController();
    pidController.setP(ClimberConstants.PID[0]);
    pidController.setI(ClimberConstants.PID[1]);
    pidController.setD(ClimberConstants.PID[2]);
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
}

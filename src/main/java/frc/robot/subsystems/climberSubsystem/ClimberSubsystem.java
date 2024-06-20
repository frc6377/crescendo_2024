package frc.robot.subsystems.climberSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motorManagement.Motor;
import frc.robot.motorManagement.MotorManager;
import frc.robot.motorManagement.MotorName;
import frc.robot.motorManagement.controlLaw.ControlLaw;
import frc.robot.motorManagement.controlLaw.PIDControlLaw;
import frc.robot.utilities.DebugEntry;

public class ClimberSubsystem extends SubsystemBase {
  private final Motor leftArmMotor;
  private final Motor rightArmMotor;

  private static final ControlLaw POSITION_CONTROL_LAW = new PIDControlLaw(); 

  private DebugEntry<Double> rightArmPoseEntry;
  private DebugEntry<Double> leftArmPoseEntry;
  private DebugEntry<Double> rightArmOutputEntry;
  private DebugEntry<Double> leftArmOutputEntry;
  private DebugEntry<String> currentCommand;

  public ClimberSubsystem() {
    leftArmMotor = MotorManager.requestMotor(MotorName.LEFT_ARM);
    rightArmMotor = MotorManager.requestMotor(MotorName.LEFT_ARM);

    rightArmPoseEntry =
        new DebugEntry<Double>(
            rightArmMotor.getPosition(), "right arm position", this);
    leftArmPoseEntry =
        new DebugEntry<Double>(rightArmMotor.getPosition(), "left arm position", this);
    rightArmOutputEntry =
        new DebugEntry<Double>(rightArmMotor.getAppliedOutput(), "right arm output", this);
    leftArmOutputEntry =
        new DebugEntry<Double>(rightArmMotor.getAppliedOutput(), "left arm output", this);
    currentCommand = new DebugEntry<String>("none", "Climber Command", this);
  }

  /**
   * Positive is up, negative is down. Is even between arms.
   *
   * @param speed the speed to set (-1 to 1)
   */
  public void applyPercent(double speed) {
    leftArmMotor.requestPercent(speed);
    rightArmMotor.requestPercent(speed);
  }

  /**
   * Positive is up, negative is down. Is even between arms.
   *
   * @param speed the speed to set (-1 to 1)
   */
  public void applyDiffertialPercent(DifferentialDemand demand) {
    leftArmMotor.requestPercent(demand.left);
    rightArmMotor.requestPercent(demand.right);
  }

  public void zeroAtCurrentPosition() {
    leftArmMotor.setPosition(0);
    rightArmMotor.setPosition(0);
  }

  public void requestTorque(double nm){
    leftArmMotor.requestTorque(nm);
    rightArmMotor.requestTorque(nm);
  }

  /**
   * Any command that uses this needs to clean up after self. As in reset min and max to -1 and 1
   *
   * @param max the maximum output (Physically 1)
   * @param min the minimum output (Physically -1)
   */
  public void setOutputLimits(double max, double min) {
    leftArmMotor.setOutputRange(min, max);
    rightArmMotor.setOutputRange(min, max);
  }

  public Position getPosition() {
    return new Position(
        leftArmMotor.getPosition(), rightArmMotor.getPosition());
  }

  public CurrentVelocity getVelocity() {
    return new CurrentVelocity(
        leftArmMotor.getVelocity(), rightArmMotor.getVelocity());
  }

  public AppliedCurrent getCurrent() {
    return new AppliedCurrent(leftArmMotor.getOutputCurrent(), rightArmMotor.getOutputCurrent());
  }

  public void requestPosition(double climbPosition) {
    leftArmMotor.requestPosition(climbPosition, POSITION_CONTROL_LAW);
    rightArmMotor.requestPosition(climbPosition, POSITION_CONTROL_LAW);
  }

  @Override
  public void periodic() {
    rightArmPoseEntry.log(rightArmMotor.getPosition());
    leftArmPoseEntry.log(leftArmMotor.getPosition());
    rightArmOutputEntry.log(rightArmMotor.getAppliedOutput());
    leftArmOutputEntry.log(leftArmMotor.getAppliedOutput());
    if (this.getCurrentCommand() != null) currentCommand.log(this.getCurrentCommand().getName());
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
}

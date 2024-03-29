package frc.robot.subsystems.climberSubsystem;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.DebugEntry;

public class ClimberSubsystem extends SubsystemBase {
  private final Servo leftServo;
  private final Servo rightServo;

  private ClimberPosition currentPosition = ClimberPosition.DEFAULT;

  private final ShuffleboardTab climberTab = Shuffleboard.getTab(this.getName());

  private DebugEntry<String> currentCommand;

  public ClimberSubsystem() {
    rightServo = new Servo(1);
    leftServo = new Servo(2);
    climberTab.add(leftServo);
    climberTab.add(rightServo);

    currentCommand = new DebugEntry<String>("none", "Climber Command", this);
  }

  protected void setMaximum() {
    currentPosition = ClimberPosition.MAXIMUM;
    updateMotors();
  }

  protected void nextStage() {
    if (currentPosition == ClimberPosition.MAXIMUM) {
      currentPosition = ClimberPosition.PREPARE_TO_CLIMB;
    } else if (currentPosition == ClimberPosition.PREPARE_TO_CLIMB) {
      currentPosition = ClimberPosition.CLIMB;
    }
    updateMotors();
  }

  private void updateMotors() {
    rightServo.setAngle(currentPosition.getAngle());
    leftServo.setAngle(currentPosition.getAngle());
  }

  @Override
  public void periodic() {
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

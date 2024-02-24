package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.utilities.DebugEntry;

import java.util.function.BooleanSupplier;

public class ClimberSubsystem extends SubsystemBase {
  final CANSparkMax leftMotorController;
  final CANSparkMax rightMotorController;
  final SparkPIDController leftPidController;
  final SparkPIDController rightPidController;
  final RelativeEncoder leftEncoder;
  final RelativeEncoder rightEncoder;
  private DebugEntry<Double> leftClimbPosition;
  private DebugEntry<Double> rightClimbPosition;
  private ShuffleboardTab ClimberTab = Shuffleboard.getTab("Climber Tab");
  double target = 0;

  public ClimberSubsystem() {
    leftMotorController = new CANSparkMax(ClimberConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotorController = new CANSparkMax(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

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
    ClimberTab.add("Left Climb PID", leftPidController).withSize(2, 2).withPosition(0, 0);
    ClimberTab.add("Right Climb PID", rightPidController).withSize(2, 2).withPosition(2, 0);
    leftClimbPosition = new DebugEntry<Double>(leftEncoder.getPosition(), "Current right climb position", this);
    rightClimbPosition = new DebugEntry<Double>(rightEncoder.getPosition(), "Current left climb position", this);
  }

  public void applyDemand(double volts) {
    leftMotorController.setVoltage(volts);
  }

  public void gotoPosition(double rotation) {
    target = rotation * Constants.ClimberConstants.GEAR_RATIO;
    leftPidController.setReference(target, ControlType.kPosition);
    rightPidController.setReference(target, ControlType.kPosition);
  }

  public Command gotoPositionCommand(double position) {
    Runnable init = () -> gotoPosition(position);
    BooleanSupplier done =
        () ->
            rightEncoder.getPosition() - target < Constants.ClimberConstants.MAX_ERROR
                && leftEncoder.getPosition() - target < Constants.ClimberConstants.MAX_ERROR;

    return new FunctionalCommand(init, () -> {}, (interupt) -> {}, done, this);
  }

  private class MinimumDelay implements BooleanSupplier {
    Timer timer;
    BooleanSupplier base;
    double wait;

    public MinimumDelay(double time, BooleanSupplier base) {
      this.base = base;
      this.wait = time;
      timer = new Timer();
    }

    @Override
    public boolean getAsBoolean() {
      timer.start();
      if (timer.advanceIfElapsed(wait)) {
        return base.getAsBoolean();
      }
      return false;
    }
  }

  @Override
  public void periodic() {
    rightClimbPosition.log(rightEncoder.getPosition());
    leftClimbPosition.log(leftEncoder.getPosition());
  }
}

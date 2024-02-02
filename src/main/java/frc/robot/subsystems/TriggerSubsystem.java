package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TriggerConstants;

public class TriggerSubsystem extends SubsystemBase {
  private CANSparkMax motor;

  public TriggerSubsystem() {
    motor = new CANSparkMax(TriggerConstants.MOTOR_ID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(40);
  }

  private void setSpeed(double speed) {
    motor.set(speed);
  }

  public Command getLoadCommand() {
    return buildCommand(TriggerConstants.LOAD_PERCENTAGE).withName("getLoadCommand");
  }

  public Command getHoldCommand() {
    return buildCommand(TriggerConstants.HOLD_PERCENTAGE).withName("getHoldCommand");
  }

  public Command getShootCommand() {
    return buildCommand(TriggerConstants.SHOOT_PERCENTAGE).withName("getShootCommand");
  }

  private Command buildCommand(double speed) {
    return new StartEndCommand(() -> setSpeed(speed), () -> setSpeed(0), this);
  }
}

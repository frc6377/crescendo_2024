package frc.robot.subsystems.triggerSubsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TriggerConstants;
import howdyutilities.DebugEntry;

public class TriggerSubsystem extends SubsystemBase {
  private CANSparkMax motor;
  private DebugEntry<String> currentCommand;
  private DebugEntry<Double> motorOutLog;

  public TriggerSubsystem() {
    motor = new CANSparkMax(TriggerConstants.MOTOR_ID, MotorType.kBrushed);
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(40);
    motor.setInverted(TriggerConstants.MOTOR_INVERT);
    currentCommand = new DebugEntry<String>("none", "Trigger Command", this);
    motorOutLog = new DebugEntry<Double>(0d, "Trigger Percent", this);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
    motorOutLog.log(speed);
  }

  public void periodic() {
    if (this.getCurrentCommand() != null) currentCommand.log(this.getCurrentCommand().getName());
  }
}

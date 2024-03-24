package frc.robot.subsystems.triggerSubsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TriggerConstants;
import frc.robot.utilities.DebugEntry;

public class TriggerSubsystem extends SubsystemBase {
  private CANSparkMax motor;
  private DebugEntry<String> currentCommand;

  public TriggerSubsystem() {
    motor = new CANSparkMax(TriggerConstants.MOTOR_ID, MotorType.kBrushed);
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(40);
    motor.setInverted(TriggerConstants.MOTOR_INVERT);
    currentCommand = new DebugEntry<String>("none", "Trigger Command", false, this);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public void periodic() {
    if (this.getCurrentCommand() != null) currentCommand.log(this.getCurrentCommand().getName());
  }
}

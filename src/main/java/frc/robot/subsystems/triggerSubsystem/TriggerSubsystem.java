package frc.robot.subsystems.triggerSubsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TriggerConstants;

public class TriggerSubsystem extends SubsystemBase {
  private CANSparkMax motor;

  public TriggerSubsystem() {
    motor = new CANSparkMax(TriggerConstants.MOTOR_ID, MotorType.kBrushed);
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(40);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }
}

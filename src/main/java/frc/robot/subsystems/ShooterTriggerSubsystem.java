package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterTriggerSubsystem extends SubsystemBase {
  private CANSparkMax shooterTriggerMotor;

  public ShooterTriggerSubsystem() {
    shooterTriggerMotor =
        new CANSparkMax(
            Constants.ShooterTriggerConstants.SHOOTER_TRIGGER_MOTOR_ID, MotorType.kBrushless);
    shooterTriggerMotor.restoreFactoryDefaults();
    shooterTriggerMotor.setSmartCurrentLimit(40);
  }

  public void setShooterTriggerSpeed(double speed) {
    shooterTriggerMotor.set(speed);
  }

  public Command getShooterTriggerLoadCommand() {
    return buildCommand(Constants.ShooterTriggerConstants.SHOOTER_TRIGGER_LOAD_PERCENTAGE);
  }

  public Command getShooterTriggerHoldCommand() {
    return buildCommand(Constants.ShooterTriggerConstants.SHOOTER_TRIGGER_HOLD_PERCENTAGE);
  }

  public Command getShooterTriggerShootCommand() {
    return buildCommand(Constants.ShooterTriggerConstants.SHOOTER_TRIGGER_SHOOT_PERCENTAGE);
  }

  private Command buildCommand(double speed) {
    return new StartEndCommand(
        () -> setShooterTriggerSpeed(speed), () -> setShooterTriggerSpeed(0), this);
  }
}

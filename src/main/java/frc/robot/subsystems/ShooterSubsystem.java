package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class ShooterSubsystem {
    private CANSparkMax shooterMotor;
    
    public ShooterSubsystem() {
        shooterMotor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setSmartCurrentLimit(40);
    }

    // Fold in speed calculation?
    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
    }
}

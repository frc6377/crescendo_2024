package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax shooterMotor;
    
    public ShooterSubsystem() {
        shooterMotor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setSmartCurrentLimit(40);
    }

    public Command shooterCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
        () -> {
          /* one-time action goes here */
        });
    }

    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
    }

    // Parameter: TargetType target ???
    public double calculateShooterSpeed(double distance) {
        double speed = distance; // Placeholder
        return speed;
    }
}

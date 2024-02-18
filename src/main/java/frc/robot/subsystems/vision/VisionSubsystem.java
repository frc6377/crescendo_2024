package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class VisionSubsystem extends SubsystemBase {
  public abstract double getTurretYaw(int id);

  public abstract double getTurretPitch(int id);
}

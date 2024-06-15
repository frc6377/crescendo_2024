package frc.robot.motorManagement.controlLaw;

import frc.robot.motorManagement.Motor;

public interface ControlLaw {
    public void bind(Motor motorType);
    public String getName();
}

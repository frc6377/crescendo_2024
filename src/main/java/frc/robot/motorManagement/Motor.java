package frc.robot.motorManagement;

import frc.robot.motorManagement.controlLaw.ControlLaw;

public interface Motor {
    /**
     * Requests the motor to output some percent of max power
     * @param x
     */
    public void requestPercent(double x);

    /**
     * Request a certain torque.
     * @param x the torque to output in newton meters
     */
    public void requestTorque(double x);
    
    /**
     * Request the motor to move the given position.
     * Using the latest control law.
     * @param pos the target position
     */
    public void requestPosition(double pos);

    /**
     * Request the motor to move the given position.
     * Using the given control law.
     * @param pos the target position
     * @param law the control law to get to the position
     */
    public default void requestPosition(double pos, ControlLaw law){
        if(!this.getControlLaw().equals(law)){
            law.bind(this);
        }
        requestPosition(pos);
    }

    /**
     * the current control law governing this motor.
     */
    public ControlLaw getControlLaw();

    /**
     * Returns current motor position in rotations.
     * Can throw runtime exception if position is not possible to get
     * @return current motor position in rotations
     */
    public double getPosition();

    /**
     * Returns current motor position in rotations per minute.
     * @return current motor position in rotations per minute (RPM)
     */
    public double getVelocity();
}

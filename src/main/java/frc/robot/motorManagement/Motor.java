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

    public Double getAppliedOutput();

    /**
     * Sets the encoder current position.
     * Does not make the motor move.
     * @param i - the rotation to set to encoder to (in rotations)
     */
    public void setPosition(double i);

    /**
     * Get the current motor output in amps.
     * @return the current motor out put in amps
     */
    public double getOutputCurrent();

    /**
     * Set the min and max motor outputs.
     * @param min - minimum motor output
     * @param max - maximum motor output 
     */
    public void setOutputRange(double min, double max);

    public void setControlLaw(ControlLaw state);
}

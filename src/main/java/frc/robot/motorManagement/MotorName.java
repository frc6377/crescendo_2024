package frc.robot.motorManagement;

import frc.robot.motorManagement.motorImplementations.NeoConfig;

public enum MotorName {
    FRONT_LEFT_STEER,
    FRONT_RIGHT_STEER,
    BACK_LEFT_STEER,
    BACK_RIGHT_STEER;
    
    public MotorType getMotorType() {
        throw new UnsupportedOperationException("Unimplemented method 'getMotorType'");
    }

    public NeoConfig getNeoConfig(){
        throw new UnsupportedOperationException("Unimplemented method 'getNeoConfig'");
    }
}

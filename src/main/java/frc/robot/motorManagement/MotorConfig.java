package frc.robot.motorManagement;

public class MotorConfig {
    private final MotorType type;
    private final int ID;
    
    public MotorConfig(MotorType type, int iD) {
        this.type = type;
        ID = iD;
    }

    public MotorType getType() {
        return type;
    }

    public int getID() {
        return ID;
    }
}

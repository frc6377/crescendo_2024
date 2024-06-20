package frc.robot.motorManagement.motorMaps;

import java.util.HashMap;
import java.util.Map;

import frc.robot.motorManagement.MotorConfig;
import frc.robot.motorManagement.MotorName;
import frc.robot.motorManagement.MotorType;

public class DefaultMotorMap {
    public static final Map<MotorName, MotorConfig> defaultMap = new HashMap<>();

    static {
        defaultMap.put(MotorName.FRONT_LEFT_STEER, new MotorConfig(MotorType.FALCON,1));
        defaultMap.put(MotorName.FRONT_RIGHT_STEER, new MotorConfig(MotorType.FALCON, 3));
        defaultMap.put(MotorName.BACK_LEFT_STEER, new MotorConfig(MotorType.FALCON, 5));
        defaultMap.put(MotorName.BACK_RIGHT_STEER, new MotorConfig(MotorType.FALCON, 7));
        

        defaultMap.put(MotorName.FRONT_LEFT_DRIVE, new MotorConfig(MotorType.FALCON, 2));
        defaultMap.put(MotorName.FRONT_RIGHT_DRIVE, new MotorConfig(MotorType.FALCON, 4));
        defaultMap.put(MotorName.BACK_LEFT_DRIVE, new MotorConfig(MotorType.FALCON, 6));
        defaultMap.put(MotorName.BACK_RIGHT_DRIVE, new MotorConfig(MotorType.FALCON, 8));
    }
}

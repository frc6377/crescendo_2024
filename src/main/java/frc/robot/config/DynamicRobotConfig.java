package frc.robot.config;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Preferences;

/**
 * The DynamicRobotConfig class keeps track of periodically 
 * changing constants on the robot
 * 
 * <p> This should only have offsets that would change with a mechanical change.
 * There should be no IDs, Conversions, or setpoint information in this config.
 */
public class DynamicRobotConfig {
    // Keys to load values from, shall not be modified under any circumstance
    private static final String frontLeftOffset_key  = "front Left Offset"; 
    private static final String frontRightOffset_key = "front Right Offset"; 
    private static final String backRightOffset_key  = "back Right Offset";  
    private static final String backLeftOffset_key   = "back Left Offset"; 

    public final double frontLeftOffset;  
    public final double frontRightOffset;
    public final double backRightOffset;
    public final double backLeftOffset;

    private DynamicRobotConfig(){
        if(Preferences.containsKey(frontLeftOffset_key)){
            frontLeftOffset = Preferences.getDouble(frontLeftOffset_key, 0d);
        }else{
            raiseWarning("Front Left Pod Offset NOT FOUND!! using default");
            frontLeftOffset = 0;
        }

        if(Preferences.containsKey(frontRightOffset_key)){
            frontRightOffset = Preferences.getDouble(frontRightOffset_key, 0d);
        }else{
            raiseWarning("Front Right Pod Offset NOT FOUND!! using default");
            frontRightOffset = 0;
        }

        if(Preferences.containsKey(backRightOffset_key)){
            backRightOffset = Preferences.getDouble(backRightOffset_key, 0d);
        }else{
            raiseWarning("Back Right Pod Offset NOT FOUND!! using default");
            backRightOffset = 0;
        }

        if(Preferences.containsKey(backLeftOffset_key)){
            backLeftOffset = Preferences.getDouble(backLeftOffset_key, 0d);
        }else{
            raiseWarning("Back Left Pod Offset NOT FOUND!! using default");
            backLeftOffset = 0;
        }
    }

    public static DynamicRobotConfig loadDynamicRobotConfig(){
        return new DynamicRobotConfig();
    }


    private static Logger DRC_logger = Logger.getLogger(DynamicRobotConfig.class.getName());
    private static void raiseWarning(String warning){
        DRC_logger.warning(warning);
    }
}

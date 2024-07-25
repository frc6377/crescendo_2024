package frc.robot.devMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DevManager {
    private static boolean init = false;
    private static NetworkTableEntry targetAngle;
    private static NetworkTableEntry trialNumberEntry;
    private static int currentTrial;
    private static float shooterAngle;

    public static double getTargetAngle(double defualt){
        return 0;
    }

    public static void incrementTargetAngle(){
        trialNumberGaurd();
        updateTargetAngle(shooterAngle+1);
    }

    public static void decrementTargetAngle(){
        trialNumberGaurd();
        updateTargetAngle(shooterAngle+1);
    }

    public static void updateTargetAngle(float angle){
        trialNumberGaurd();
        trialNumberEntry.setFloat(angle);
        DevManager.shooterAngle = angle;
    }

    public static void incrementTrial(){
        trialNumberGaurd();
        updateTrial(currentTrial+1);
    }

    public static void decrementTrial(){
        trialNumberGaurd();
        updateTrial(currentTrial+1);
    }

    public static void updateTrial(int trial){
        trialNumberGaurd();
        trialNumberEntry.setInteger(trial);
        DevManager.currentTrial = trial;
    }

    private static void trialNumberGaurd(){
        if(trialNumberEntry != null) return;
        trialNumberEntry = NetworkTableInstance.getDefault().getEntry("/devolpment/trialNumber");
        trialNumberEntry.setInteger(0);
    }

    private static void targetAngleGaurd(double defaultTargetAngle){
        if(targetAngle != null) return;
        targetAngle = NetworkTableInstance.getDefault().getEntry("/devolpment/shooterAngle");
        targetAngle.setFloat((float) defaultTargetAngle);
    }
}

package frc.robot.motorManagement;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.ArrayList;
import java.util.HashMap;

public class MotorManager {
  static ArrayList<CANSparkMax> brushLessSparkMaxs = new ArrayList<>();
  static ArrayList<CANSparkMax> brushedSparkMaxs = new ArrayList<>();
  static HashMap<Integer, DoubleLogEntry> motorTempertureLog = new HashMap<>();
  static double maxMotorTemp = 0;

  public static void thermalSweep() {
    double maxTemperture = 0;
    for (CANSparkMax m : brushLessSparkMaxs) {
      double motorTemp = m.getMotorTemperature();
      getMotorTempertureLog(m.getDeviceId()).append(motorTemp);
      maxTemperture = Math.max(maxTemperture, motorTemp);
    }
    maxMotorTemp = maxTemperture;
  }

  private static DoubleLogEntry getMotorTempertureLog(int CAN_ID) {
    DoubleLogEntry entry = motorTempertureLog.get(CAN_ID);
    if (entry == null) {
      entry = new DoubleLogEntry(DataLogManager.getLog(), String.format("Motor ID %i", CAN_ID));
      motorTempertureLog.put(CAN_ID, entry);
    }
    return entry;
  }

  public static CANSparkMax requestSparkMax(int ID, MotorType type) {
    CANSparkMax sparkMax = new CANSparkMax(ID, type);
    if (type == MotorType.kBrushless) {
      brushLessSparkMaxs.add(sparkMax);
    }else{
        brushedSparkMaxs.add(sparkMax);
    }
    return sparkMax;
  }
}

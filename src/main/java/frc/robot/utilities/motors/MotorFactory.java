package frc.robot.utilities.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import java.util.ArrayList;
import java.util.List;

public class MotorFactory {
  private static List<CANSparkMax> CANSparkMaxs = new ArrayList<>();
  private static List<TalonFX> talonFXs = new ArrayList<>();

  protected static List<Integer> getRevIds() {
    List<Integer> sparkMaxIds =
        CANSparkMaxs.stream().map((sparkMax) -> sparkMax.getDeviceId()).toList();
    return sparkMaxIds;
  }

  protected static List<Integer> getCTREIds() {
    List<Integer> talonFXIds = talonFXs.stream().map((talonfx) -> talonfx.getDeviceID()).toList();
    return talonFXIds;
  }

  public TalonFX getTalonFX(int id) {
    return getTalonFX(id, "");
  }

  public TalonFX getTalonFX(int id, String busName) {
    TalonFX talonFX = new TalonFX(id, busName);
    talonFXs.add(talonFX);

    return talonFX;
  }
}

package frc.robot.motorManagement;

public class ThermalLimits {
    private final double warningTempC;
    private final double thermalShutdownTempC;

    private static final ThermalLimits neoDefaultThermalLimits = new ThermalLimits(80, 100);

    private ThermalLimits(double warningTemp, double thermalShutdownTemp) {
        this.warningTempC = warningTemp;
        this.thermalShutdownTempC = thermalShutdownTemp;
    }

    public ThermalLimits neoDefaultThermalLimits(){
        return neoDefaultThermalLimits;
    }

    public double getThermalShutdownTempC() {
        return thermalShutdownTempC;
    }

    public double getWarningTempC() {
        return warningTempC;
    }

}

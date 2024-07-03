package frc.robot.motorManagement.motorImplementations;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.motorManagement.Motor;
import frc.robot.motorManagement.MotorName;
import frc.robot.motorManagement.MotorType;
import frc.robot.motorManagement.controlLaw.ControlLaw;

public class NeoMotor implements Motor{
    private final CANSparkMax sparkMax;
    private final NeoConfig motorCfg;

    public NeoMotor(EventLoop motorUpdateLoop, MotorName motorName){
        if(motorUpdateLoop == null){
            DriverStation.reportWarning(motorName.name()+" will not have thermal limits and control law behavior will be unpredictable", false);
        }
        
        if(motorName.getMotorType() != MotorType.NEO){
            throw new RuntimeException("Invalid motor construction. Attempted to make a neo from a "+motorName.getMotorType()+" motor");
        }

        motorCfg = motorName.getNeoConfig();

        sparkMax = new CANSparkMax(motorCfg.CAN_ID(), motorCfg.motorType());
        motorUpdateLoop.bind(this::update);
    }

    private void update(){
        sparkMax.getMotorTemperature();
    }

    @Override
    public void requestPercent(double x) {
        sparkMax.set(x);
    }

    @Override
    public void requestTorque(double x) {
        double maxTorque = getMaxTorqueAtRPM(this.getVelocity());
        if(maxTorque == 0){
            sparkMax.set(Math.copySign(1, x));
        }else{
            sparkMax.set(x/maxTorque);
        }
    }

    private static double getMaxTorqueAtRPM(double RPM){
        // Equation based on https://motors.vex.com/other-motors/neo
        return MathUtil.clamp(3.28 *  (Math.abs(RPM)/5820), 0, 3.28);
    }

    @Override
    public void requestPosition(double pos) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'requestPosition'");
    }

    @Override
    public ControlLaw getControlLaw() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getControlLaw'");
    }

    @Override
    public double getPosition() {
        return sparkMax.getEncoder().getPosition();
    }

    @Override
    public double getVelocity() {
        return sparkMax.getEncoder().getVelocity();
    }
}

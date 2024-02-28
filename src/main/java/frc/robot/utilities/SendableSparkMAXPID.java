package frc.robot.utilities;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableSparkMAXPID implements Sendable {

  private final SparkPIDController sparkPID;

  public SendableSparkMAXPID(SparkPIDController pid) {
    sparkPID = pid;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("kP", sparkPID::getP, sparkPID::setP);
    builder.addDoubleProperty("ki", sparkPID::getI, sparkPID::setI);
    builder.addDoubleProperty("kd", sparkPID::getD, sparkPID::setD);
    throw new UnsupportedOperationException("Unimplemented method 'initSendable'");
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final BiConsumer<Pose2d, Double> measurementConsumer;


    public LimelightSubsystem(BiConsumer<Pose2d, Double> measurementConsumer) {
        this.measurementConsumer = measurementConsumer;

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    }

    public double getTagCount() {
        return 2; //TODO: find how to pull tag count from network tables
    }

    private Pose2d getPose2dFromArray(double[] botpose){
        return new Pose2d(botpose[0], botpose[1], new Rotation2d(botpose[5]));
    }

    private double getTimeFromArray(double[] botpose){
        //Accounts for latency (https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization)
        return Timer.getFPGATimestamp() - (botpose[6]/1000.0);
    }

    @Override
    public void periodic() {
        if(getTagCount() > 1){
            double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
            measurementConsumer.accept(
                getPose2dFromArray(botpose), 
                getTimeFromArray(botpose)
            );
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}

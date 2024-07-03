package frc.robot.motorManagement.motorImplementations;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.motorManagement.ThermalLimits;

public record NeoConfig(int CAN_ID, MotorType motorType, ThermalLimits thermalLimits) {
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Add your docs here. */
public class HowdyFF {
  private double kS;
  private double kG;
  private double kV;
  private double kA;

  public HowdyFF(double ks, double kg, double kv, double ka) {
    this.kS = ks;
    this.kG = kg;
    this.kV = kv;
    this.kA = ka;
  }

  public double getKS() {
    return this.kS;
  }

  public double getKG() {
    return this.kG;
  }

  public double getKV() {
    return this.kV;
  }

  public double getKA() {
    return this.kA;
  }
}

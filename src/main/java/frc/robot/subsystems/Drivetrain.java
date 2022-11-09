// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANSparkMax spark_1;
  public Drivetrain() {
      spark_1 = new CANSparkMax(0, MotorType.kBrushless);
  }
  /*public void sparkLeft() {
    spark_1.set(-0.05);
  }
  public void sparkRight() {
    spark_1.set(0.05);
  }  
  public void StopTurn() {
    spark_1.set(0);
  }*/
  public void turn(double speed){
    spark_1.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

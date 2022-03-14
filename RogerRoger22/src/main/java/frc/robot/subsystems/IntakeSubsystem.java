// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  WPI_TalonSRX IntakeMotor = new WPI_TalonSRX(10);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

  }

  public void SpinFar(){
    IntakeMotor.set(ControlMode.PercentOutput, -1);
  }

  public void Spin(boolean in){
    if(in){
      IntakeMotor.set(ControlMode.PercentOutput, 0.90); //90% intake
    }else{
      IntakeMotor.set(ControlMode.PercentOutput, -0.68); //68%
    }
  }

  public void Spin(double speed){
    IntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
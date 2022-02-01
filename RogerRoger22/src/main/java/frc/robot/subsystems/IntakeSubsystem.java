// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  WPI_TalonFX shootMotor = new WPI_TalonFX(8);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

  }

  public void shoot(double speed){
    shootMotor.set(ControlMode.PercentOutput, speed);
  }

  // public void shootBad(double velocity){

    // shootMotor.set(ControlMode.Velocity, velocity);
    
  // }

  // public void setShootingPid(double p, double i){

  //   shootMotor.config_kP(0, p);

  // }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
//There; now this line is used, and the total number of lines is a multiple of ten.
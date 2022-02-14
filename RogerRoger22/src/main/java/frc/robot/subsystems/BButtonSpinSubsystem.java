// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BButtonSpinSubsystem extends SubsystemBase {

  // WPI_TalonFX spinMotor = new WPI_TalonFX(8);

  /** Creates a new BButtonSpinSubsystem. */
  public BButtonSpinSubsystem() {

  }

public void SetPIDController(){

  // spinMotor.setSelectedSensorPosition(0);

  // spinMotor.config_kP(0, 0.05);
  // spinMotor.config_kI(0, 0);
  // spinMotor.config_kD(0, 0);
  // spinMotor.config_kF(0, 0);
}

  // public void spin(double speed){
  //   spinMotor.set(ControlMode.PercentOutput, speed);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

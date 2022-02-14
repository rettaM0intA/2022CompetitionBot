// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RandomSubsystem extends SubsystemBase {

  // WPI_TalonFX shootMotor = new WPI_TalonFX(10);

  /** Creates a new RandomSubsystem. */
  public RandomSubsystem() {



  }


  public void shoot(double speed){
    // shootMotor.set(ControlMode.PercentOutput, speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MoveAction;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeMoverSubsystem extends SubsystemBase {

  CANSparkMax motor = new CANSparkMax(10, MotorType.kBrushless);
  DigitalInput topLimit = new DigitalInput(0);
  DigitalInput bottomLimit = new DigitalInput(1);

  /** Creates a new IntakeMoverSubsystem. */
  public IntakeMoverSubsystem() {

  }

  public void Move(boolean moveUp){
    if(moveUp){
      if(topLimit.get()){
        motor.set(0); //Make the intake stay up
      }else{
        motor.set(0); //Make the intake move upward
      }
    }else{
      if(bottomLimit.get()){
        motor.set(0); //Make the intake stay down. Probably will stay 0
      }else{
        motor.set(0); //Make the intake move down slowly.
      }
    }
  }

  public void Move(boolean moveUp, double speed){
    if(moveUp){
      if(topLimit.get()){
        motor.set(0); //Make the intake stay up
      }else{
        motor.set(speed); //Make the intake move upward
      }
    }else{
      if(bottomLimit.get()){
        motor.set(0); //Make the intake stay down. Probably will stay 0
      }else{
        motor.set(speed); //Make the intake move down slowly.
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

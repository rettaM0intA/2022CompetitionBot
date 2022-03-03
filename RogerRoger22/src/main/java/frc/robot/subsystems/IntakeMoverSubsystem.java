// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO negative is down, positive is up

package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MoveAction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeMoverSubsystem extends SubsystemBase {

  CANSparkMax motor = new CANSparkMax(14, MotorType.kBrushless);

  boolean pidSet = false;
  double highestPower;

  /** Creates a new IntakeMoverSubsystem. */
  public IntakeMoverSubsystem() {
    motor.setIdleMode(IdleMode.kBrake);

    if(!pidSet){
      SetPID();
    }

  }

  public boolean Move(boolean moveUp){
    
    if(moveUp){
      motor.getPIDController().setReference(0, ControlType.kPosition);
      if(motor.getEncoder().getPosition() > -1){
        return true;
      }
    }
    if(!moveUp){
      motor.getPIDController().setReference(-56, ControlType.kPosition);
      if(motor.getEncoder().getPosition() < -55){
        return true;
      }
    }



    // if(highestPower < motor.getBusVoltage()){
    //   highestPower = motor.getBusVoltage();
    // }
    // if(moveUp){
    //   if(motor.getEncoder().getPosition() > -10){
    //     motor.set(0.05);
    //   }else{
    //     motor.set(0.20);
    //   }
      
    // }else{
    //   motor.set(-0.15);
    // }
    return false;
  }

  public void Move(boolean moveUp, double speed){
    if(moveUp){
      motor.set(speed);
    }else{
      motor.set(-speed);
    //   if(topLimit.get()){
    //     motor.set(0); //Make the intake stay up
    //   }else{
    //     motor.set(speed); //Make the intake move upward
    //   }
    // }else{
    //   if(bottomLimit.get()){
    //     motor.set(0); //Make the intake stay down. Probably will stay 0
    //   }else{
    //     motor.set(speed); //Make the intake move down slowly.
    //   }
    }
  }

  public void SetPID(){
    motor.getPIDController().setP(0.066);
    motor.getPIDController().setI(0.0000005);
    motor.getPIDController().setD(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("IntakeMover Position", motor.getEncoder().getPosition());
    // SmartDashboard.putNumber("highest speed", highestPower);
  }
}

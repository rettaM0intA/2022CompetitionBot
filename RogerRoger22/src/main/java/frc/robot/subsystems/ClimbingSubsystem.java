// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {

  CANSparkMax rightClimbMotor = new CANSparkMax(12, MotorType.kBrushless);
  // CANSparkMax leftClimbMotor = new CANSparkMax(13, MotorType.kBrushless);
  

  /** Creates a new ClimbingSubsystem. */
  public ClimbingSubsystem() {
    rightClimbMotor.setIdleMode(IdleMode.kBrake);
  }

  //right max 202 but motor limit should be 190. momentum

  public void Climb(boolean isUp){
    if(isUp){
<<<<<<< Updated upstream
      rightClimbMotor.set(0.1);
      // leftClimbMotor.set(0);
    }else{
      rightClimbMotor.set(-0.1);
=======
      if(rightClimbMotor.getEncoder().getPosition() < 190){
        rightClimbMotor.set(1); //100%
      }else{
        rightClimbMotor.set(0);
      }
      if(leftClimbMotor.getEncoder().getPosition() > -200){
        leftClimbMotor.set(-1); //100%
      }else{
        leftClimbMotor.set(0);
      }
    }else{
      if(rightClimbMotor.getEncoder().getPosition() > 4){
        rightClimbMotor.set(-0.5); //50%
      }else{
        rightClimbMotor.set(0);
      }
      if(leftClimbMotor.getEncoder().getPosition() < -4){
        leftClimbMotor.set(0.5); //50%
      }else{
        leftClimbMotor.set(0);
      }
      
>>>>>>> Stashed changes
    }
  }

  public void Hold(){
    rightClimbMotor.set(0);
    // leftClimbMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climber rotations", leftClimbMotor.getEncoder().getPosition());
    
  }
}

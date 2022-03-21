// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Add inputs to the constructor. Done!
// Add variables to copy inputs. Done!
// Add if statement that checks if is at right angle and sets isFinished to true.
// Add movement logic (probably gonna be a bunch of if statements).

//made by Drew and Carter inc. Patent Pending
// Notes on things changed w/Cody's help:
/*set inputs to variables from variable declaration

spinToPoint(speed, 0, 0)*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoChassisSpinPID extends CommandBase {

  boolean isFinished = false;
  boolean isRight = false;
  // boolean isInit = false;
  double speed = 0; // Corresponds with input.
  double goalDegree = 0; // Corresponds with input.
  double currentDegree = 0;
  int buffer = 0;
  
  // AHRS gyro;

  /** Creates a new AutoChassisSpinPID. */
  /**
   * How we turn the robot
   * @param m_goalDegree What direction you want to go to in degrees.
   * @param m_speed How fast you want to move in percent.
   */
  public AutoChassisSpinPID(double m_goalDegree, double m_speed, boolean m_isRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_chassisSubsystem);
    speed = m_speed / 100;
    isRight = m_isRight;

    if(isRight){
      goalDegree = m_goalDegree;
    }else{
      goalDegree = -m_goalDegree;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // gyro = RobotContainer.m_chassisSubsystem.gyro;

    // RobotContainer.m_chassisSubsystem.resetGyro();
    RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0);
    // buffer = 0;
    // Commented this out^.
    isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    buffer += 1;

    currentDegree = RobotContainer.m_chassisSubsystem.gyro.getAngle();

    // if(goalDegree > 0){
    if(isRight){
      if(buffer > 20){
        RobotContainer.m_chassisSubsystem.driveTeleop(0, 0, -speed);
      }else{
        RobotContainer.m_chassisSubsystem.driveTeleop(0, 0, -0.03);
      }
    
      if(currentDegree > goalDegree -35 && speed > 0.2){
        speed *= 0.5;
      }

      if(currentDegree >= goalDegree){
        isFinished = true;
      }

    }else{
        if(buffer > 20){
          RobotContainer.m_chassisSubsystem.driveTeleop(0, 0, speed);
        }else{
          RobotContainer.m_chassisSubsystem.driveTeleop(0, 0, 0.03);
        }
      
        if(currentDegree < goalDegree + 35 && speed > 0.2){
          speed *= 0.5;
        }
  
        if(currentDegree <= goalDegree){
          isFinished = true;
        }
  
      }

  }

  
    
  //   if(goalDegree > 0 && gyro.getAngle() > goalDegree -35 && speed > 0.03){
  //     speed *= 0.5;
  //   }else if(goalDegree > 0 && gyro.getAngle() < goalDegree +35 && speed > 0.03){
  //     speed *= 0.5;
  //   }

  //   if(goalDegree > 0 && gyro.getAngle() >= goalDegree){
  //     isFinished = true;
  //   }else if(goalDegree < 0 && gyro.getAngle() <= goalDegree){
  //     isFinished = true;
  //   }

  // }


    // This^ still needs to be tested.

      // if (currentDegree < goalDegree) {
      //   RobotContainer.m_chassisSubsystem.spinToPoint(speed, 0, 0);
      // } else if (currentDegree > goalDegree) {
      //   RobotContainer.m_chassisSubsystem.spinToPoint(-speed, 0, 0);
      // } else {
      //   isFinished = true;
      // }
    // }
  // }
    // Commented this out^.


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished){
      // RobotContainer.m_chassisSubsystem.resetGyro();
      RobotContainer.m_chassisSubsystem.zeroMotors();
      RobotContainer.m_chassisSubsystem.driveTeleop(0, 0, 0);
      isFinished = false;
      return true;
    }
    return false;
  }
}

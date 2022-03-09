// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoChassisMovePid extends CommandBase {

  boolean forward;
  boolean isFinished = false;
  double goalRadian;
  double speed;
  double fwd;
  double strafe;
  int waitBeforeStart;

  AHRS gyro;

  double goalDistance;
  double fLgoalDistance;
  double fRgoalDistance;
  double bLgoalDistance;
  double bRgoalDistance;

  double countsCorrect;
  double averageDistance;

  double fLspeed;
  double fRspeed; 
  double bLspeed;
  double bRspeed;

  double smallChange = 0.00000001;
  double bigChange = 0.000001;

  int direction = 1;
  int timesTurned = 0;

  /** Creates a new AutoChasssisMovePid.
   * 
   * @param m_degree What direction you want to go in degrees.
   * @param m_speed How fast you want to move in percent.
   * @param m_distance How far you want to go in feet
   * 
   */
  public AutoChassisMovePid(double m_degree, double m_speed, double m_distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_chassisSubsystem);

    goalRadian = ((m_degree + 90) * Math.PI / 180); //The math requires radians, so translate degree input to radians
    speed = m_speed / 100;

    
    fwd = (Math.sin(goalRadian) / 100) * speed;
    strafe = (Math.cos(goalRadian) / 100) * speed;

    goalDistance = m_distance * Constants.kChassisEstimatedRotationsToInches * 12;
    fLgoalDistance = m_distance * Constants.kChassisEstimatedRotationsToInches * 12;
    fRgoalDistance = m_distance * Constants.kChassisEstimatedRotationsToInches * 12;
    bLgoalDistance = m_distance * Constants.kChassisEstimatedRotationsToInches * 12;
    bRgoalDistance = m_distance * Constants.kChassisEstimatedRotationsToInches * 12;

    fLspeed = speed;
    fRspeed = speed;
    bLspeed = speed;
    bRspeed = speed;

    if(goalDistance > 0){
      forward = true;
    }else{
      forward = false;
      smallChange *= -1;
      bigChange *= -1;
    }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.m_chassisSubsystem.SetPIDCheckerGoals(fLgoalDistance, fRgoalDistance, bLgoalDistance, bRgoalDistance);
    RobotContainer.m_chassisSubsystem.zeroMotors();
    RobotContainer.m_chassisSubsystem.resetGyro();
    RobotContainer.m_chassisSubsystem.wheelBrakeMode();
    isFinished = false;
    waitBeforeStart = 0;
    gyro = RobotContainer.m_chassisSubsystem.gyro;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    

    waitBeforeStart += 1;

    if(waitBeforeStart > 30){
      RobotContainer.m_chassisSubsystem.driveToPoint(fwd, strafe, 0, -fLspeed, -fRspeed, -bLspeed, -bRspeed);
    }else if(waitBeforeStart > 12){
      RobotContainer.m_chassisSubsystem.driveToPoint(fwd, strafe, 0, 0.01, 0.01, 0.01, 0.01);
    }else{
      RobotContainer.m_chassisSubsystem.driveToPoint(fwd, strafe, 0, 0, 0, 0, 0);
    }
    
    // if((RobotContainer.m_chassisSubsystem.wheelMotorCountAverage() > goalDistance && goalDistance > 0) || (RobotContainer.m_chassisSubsystem.wheelMotorCountAverage() < goalDistance && goalDistance < 0)){

    //   speed *= 0.001;
    //   fLspeed = -speed;
    //   fRspeed = -speed;
    //   bLspeed = -speed;
    //   bRspeed = -speed;

    //   timesTurned += 1;
    //   bigChange *= -1;
    //   smallChange *= -1;

    // }

    if(forward){

    if(gyro.getAngle() > 1){
      if(gyro.getAngle() > 3){
        fRspeed += bigChange;
        bRspeed += bigChange;

        fLspeed -= bigChange / 2;
        bLspeed -= bigChange / 2;
      }else{
        fRspeed += smallChange;
        bRspeed += smallChange;

        fLspeed -= smallChange / 2;
        bLspeed -= smallChange / 2;
      }
    }else if(gyro.getAngle() < -1){
      if(gyro.getAngle() < -3){

        fLspeed += bigChange;
        bLspeed += bigChange;

        fRspeed -= bigChange / 2;
        bRspeed -= bigChange / 2;

      }else{

        fLspeed += smallChange;
        bLspeed += smallChange;

        fRspeed -= smallChange / 2;
        bRspeed -= smallChange / 2;

      }
    }

    }else{

      //TODO, switch motor pairs

      if(gyro.getAngle() > 1 * direction){
        if(gyro.getAngle() > 3 * direction){
          fRspeed -= bigChange;
          bRspeed -= bigChange;
  
          fLspeed += bigChange / 2;
          bLspeed += bigChange / 2;
        }else{
          fRspeed -= smallChange;
          bRspeed -= smallChange;
  
          fLspeed += smallChange / 2;
          bLspeed += smallChange / 2;
        }
      }else if(gyro.getAngle() < -1 * direction){
        if(gyro.getAngle() < -3 * direction){
  
          fLspeed -= bigChange;
          bLspeed -= bigChange;
  
          fRspeed += bigChange / 2;
          bRspeed += bigChange / 2;
  
        }else{
  
          fLspeed -= smallChange;
          bLspeed -= smallChange;
  
          fRspeed += smallChange / 2;
          bRspeed += smallChange / 2;
  
        }
      }
      
    }

    if(forward && RobotContainer.m_chassisSubsystem.wheelMotorCountAverage() > goalDistance - (Constants.kChassisEstimatedRotationsToInches * 12)){
      fLspeed *= 0.99;
      fRspeed *= 0.99;
      bLspeed *= 0.99;
      bRspeed *= 0.99;
    }else if(RobotContainer.m_chassisSubsystem.wheelMotorCountAverage() < goalDistance + (Constants.kChassisEstimatedRotationsToInches * 12)){
      fLspeed *= 0.99;
      fRspeed *= 0.99;
      bLspeed *= 0.99;
      bRspeed *= 0.99;
    }

    // if((RobotContainer.m_chassisSubsystem.wheelMotorCountAverage() == goalDistance || (RobotContainer.m_chassisSubsystem.checkPIDlocation() && waitBeforeStart > 31)) && timesTurned >= 4){
    //   countsCorrect += 1;
    // }else{
    //   countsCorrect = 0;
    // }

    if(forward && RobotContainer.m_chassisSubsystem.wheelMotorCountAverage() > goalDistance){
      isFinished = true;
    }else if(!forward && RobotContainer.m_chassisSubsystem.wheelMotorCountAverage() < goalDistance){
      isFinished = true;
    }

    // if(countsCorrect > 5 && waitBeforeStart > 30){
    //   isFinished = true;
    // }
    
    RobotContainer.m_chassisSubsystem.SetPIDCheckerGoals(fLgoalDistance, fRgoalDistance, bLgoalDistance, bRgoalDistance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished){
      RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0);
      RobotContainer.m_chassisSubsystem.resetGyro();
      RobotContainer.m_chassisSubsystem.zeroMotors();
      //RobotContainer.m_chassisSubsystem.disablePids();
      SmartDashboard.putBoolean("gun", true);

      return true;
    }

    SmartDashboard.putBoolean("gun", false);
    return false;
  }
}
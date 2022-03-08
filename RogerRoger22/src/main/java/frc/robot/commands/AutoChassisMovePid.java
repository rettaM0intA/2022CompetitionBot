// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoChassisMovePid extends CommandBase {

  boolean isFinished;
  double goalRadian;
  double speed;
  double fwd;
  double strafe;
  int waitBeforeStart;
  double goalDistance;

  double countsCorrect;
  double averageDistance;

  /** Creates a new AutoChasssisMovePid.
   * 
   * @param m_degree What direction you want to go in degrees.
   * @param m_speed How fast you want to move in percent.
   * @param m_distance
   * 
   */
  public AutoChassisMovePid(double m_degree, double m_speed, double m_distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_chassisSubsystem);

    // goalRadian = ((m_degree + 90) * Math.PI / 180); //The math requires radians, so translate degree input to radians
    // speed = m_speed;

    
    // fwd = (Math.sin(goalRadian) / 100) * speed;
    // strafe = (Math.cos(goalRadian) / 100) * speed;

    fwd = 0.1;
    strafe = 0;

    goalDistance = m_distance * Constants.kChassisEstimatedRotationsToInches * 12;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.m_chassisSubsystem.zeroMotors();
    isFinished = false;
    waitBeforeStart = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    waitBeforeStart += 1;

    if(waitBeforeStart > 10){
      RobotContainer.m_chassisSubsystem.driveToPoint(0.5, 0, 0, goalDistance);
    
    }

    if(RobotContainer.m_chassisSubsystem.wheelMotorCountAverage() == goalDistance || (RobotContainer.m_chassisSubsystem.checkPIDlocation() && waitBeforeStart > 11)){
      countsCorrect += 1;
    }else{
      countsCorrect = 0;
    }

    if(countsCorrect > 5 && waitBeforeStart > 10){
      isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished){
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
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AutoChassisSpinPID extends CommandBase {

  boolean isFinished;

  /** Creates a new AutoChassisSpinPID. */
  public AutoChassisSpinPID() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_chassisSubsystem.resetGyro();
    RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    //RobotContainer.m_chassisSubsystem.driveToPoint(0, 0, .001, fLgoalPosition, fRgoalPosition, bLgoalPosition, bRgoalPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isFinished){
      RobotContainer.m_chassisSubsystem.resetGyro();
      RobotContainer.m_chassisSubsystem.zeroMotors();
      //RobotContainer.m_chassisSubsystem.disablePids();

      return true;
    }
    return false;
  }
}

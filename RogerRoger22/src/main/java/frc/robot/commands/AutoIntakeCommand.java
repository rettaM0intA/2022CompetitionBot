// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IntakeDirection;
import frc.robot.RobotContainer;

public class AutoIntakeCommand extends CommandBase {

  boolean finished = false;
  boolean direction;
  boolean isClose;
  double startTime;
  double goalTime;
  Timer timer = new Timer();

  /** Creates a new AutoIntakeCommand. 
   * @param SpinIn IntakeDirection either in (true) or out (false)
   * @param m_time amount of seconds it should be on for
   * @param m_isClose True if next to the goal, false if not.
  */
  public AutoIntakeCommand(Boolean SpinIn, double m_time, Boolean m_isClose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intakeSubsystem);
    direction = SpinIn;
    goalTime = m_time;
    isClose = m_isClose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = timer.get();
    RobotContainer.m_intakeSubsystem.Spin(0);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(isClose){
      RobotContainer.m_intakeSubsystem.Spin(direction);
    }else{
      RobotContainer.m_intakeSubsystem.Spin(-1);
    }

    if(goalTime <= timer.get() - startTime){
      finished = true;
    }


    SmartDashboard.putNumber("Time", timer.get());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intakeSubsystem.Spin(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(finished){
      RobotContainer.m_intakeSubsystem.Spin(0);
      return true;
    }
    return false;
  }
}

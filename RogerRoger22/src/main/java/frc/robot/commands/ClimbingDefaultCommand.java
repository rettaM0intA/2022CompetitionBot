// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControllerInControl;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimbingDefaultCommand extends CommandBase {
  /** Creates a new ClimbingDefaultCommand. */
  public ClimbingDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_climbingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.gamepadDriver == ControllerInControl.MainController){
      if(RobotContainer.operator.getRightY() > 0.05){
        RobotContainer.m_climbingSubsystem.Climb(true);
      }else if(RobotContainer.operator.getRightY() < -0.05){
        RobotContainer.m_climbingSubsystem.Climb(false);
      }
    }else{
      RobotContainer.m_climbingSubsystem.Hold();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

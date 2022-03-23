// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IntakeDirection;
import frc.robot.RobotContainer;

public class IntakeMoverMoveCommand extends CommandBase {
  /** Creates a new IntakeMoverMoveCommand. */
  public IntakeMoverMoveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intakeMoverSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.intakeDirection == IntakeDirection.up){
      RobotContainer.m_intakeMoverSubsystem.Move(true);
    }else if(RobotContainer.intakeDirection == IntakeDirection.down){
      RobotContainer.m_intakeMoverSubsystem.Move(false);
    }

    if(RobotContainer.m_intakeMoverSubsystem.Move(true)){
      
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intakeMoverSubsystem.Move(true, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

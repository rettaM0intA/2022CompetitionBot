// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IntakeDirection;
import frc.robot.RobotContainer;

public class IntakeMoverDefaultCommand extends CommandBase {

  int stillCounter = 0;

  /** Creates a new IntakeMoverDefaultCommand. */
  public IntakeMoverDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intakeMoverSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(RobotContainer.intakeDirection == IntakeDirection.up){
    //   RobotContainer.m_intakeMoverSubsystem.Move(true);
    // }else{
    //   RobotContainer.m_intakeMoverSubsystem.Move(false);
    // }

    if(RobotContainer.intakeDirection != IntakeDirection.still){
      if(RobotContainer.intakeDirection == IntakeDirection.up){
        if(RobotContainer.m_intakeMoverSubsystem.Move(true)){
          stillCounter += 1;
        }
      }else{
        if(RobotContainer.m_intakeMoverSubsystem.Move(false)){
          RobotContainer.intakeDirection = IntakeDirection.still;
        }
      }

      if(stillCounter >= 2){
        RobotContainer.intakeDirection = IntakeDirection.still;
      }

    }else{
      if(RobotContainer.chosenDirection == IntakeDirection.down){
        RobotContainer.m_intakeMoverSubsystem.Move(false, 0.03);
      }else{

        stillCounter = 0;
        RobotContainer.m_intakeMoverSubsystem.Move(true, 0.05);
      }
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

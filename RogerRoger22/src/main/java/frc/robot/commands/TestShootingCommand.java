// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TestShootingCommand extends CommandBase {
  /** Creates a new TestShootingCommand. */
  public TestShootingCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_TestShootingCommand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.m_RandomSubsystem.shoot(1);
    RobotContainer.m_IntakeSubsystem.shoot(1);

    // if(I can remember){
    //   be suprised
    // }else{
    //   be disapointed
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_RandomSubsystem.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
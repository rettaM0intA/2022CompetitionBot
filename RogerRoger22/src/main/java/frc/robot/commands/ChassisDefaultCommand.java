// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.ControllerInControl;
import frc.robot.RobotContainer;

public class ChassisDefaultCommand extends CommandBase {
  /** Creates a new ChassisDefaultCommand. */
  public ChassisDefaultCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Checks which controller is currently driver
<<<<<<< Updated upstream
    if(RobotContainer.gamepadDriver == ControllerInControl.gamepad){
    //XBox controller
    RobotContainer.m_chassisSubsystem.driveTeleop(-RobotContainer.operator.getLeftY(), -RobotContainer.operator.getLeftX(), -RobotContainer.operator.getRightX()*.25);
    }else{
    //Joystick
    RobotContainer.m_chassisSubsystem.driveTeleop(-RobotContainer.driver.getY(), -RobotContainer.driver.getX(), -RobotContainer.driver.getTwist() * .25);
    }
=======
    // if(RobotContainer.gamepadDriver == ControllerInControl.MainController){
    // //XBox controller
    // RobotContainer.m_chassisSubsystem.driveTeleop(-RobotContainer.operator.getLeftY(), -RobotContainer.operator.getLeftX(), -RobotContainer.operator.getRightX()*.25);
    // }else{
    //XBox controller
    RobotContainer.m_chassisSubsystem.driveTeleop(-RobotContainer.driver.getLeftY(), -RobotContainer.driver.getLeftX(), -RobotContainer.driver.getRightX() * .25);
    // }
>>>>>>> Stashed changes


    
    SmartDashboard.putNumber("distancetraveld", RobotContainer.m_chassisSubsystem.wheelMotorCountAverage() / Constants.kChassisEstimatedRotationsToInches);
  }

  // Called once the command ends or is interrupted.
  //@Override
  //public void end(Boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

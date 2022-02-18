// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// "Nobody cares about laws and stuff." -Drew Jezowski, February 18, 2022
// "Let 'em make mashed potatoes." -Drew Jezowski, February 18, 2022
// "I don't know any other quotes." -Drew Jezowski, February 18, 2022
// "Hey, I think he's infected with the quote virus." -Drew Jezowski, February 18, 2022
// "Yeah, you guys are supposed to be helping each other make code, not quotes." -Cody VanSnepson, February 18, 2022
// "Had to include that last part, didn't you?" -Cody VanSnepson, February 18, 2022
/** "'How's the command coming?' 
'It's not.'" -Cody VanSnepson & Carter Davis, February 18, 2022 */
// "You're not more than dead, you're alive!" -Cody VanSnepson, February 18, 2022
// "Hay? That's for horses!" -Cody VanSnepson, February 18, 2022
// "Doce trece or catorce, why not?" -Cody VanSnepson, February 18, 2022

// Add inputs to the constructor. Done!
// Add variables to copy inputs. Done!
// Add if statement that checks if is at right angle and sets isFinished to true.
// Add movement logic (probably gonna be a bunch of if statements).

//made by Drew and Carter inc. Patent Pending

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoChassisSpinPID extends CommandBase {

  boolean isFinished = false;
  boolean isInit = false;
  double speed = 0; // Corresponds with input.
  double goalDegree = 0; // Corresponds with input.
  double currentDegree = 0;
  int buffer = 0;

  /** Creates a new AutoChassisSpinPID. */
  public AutoChassisSpinPID(double m_goalDegree, double m_speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_chassisSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.m_chassisSubsystem.resetGyro();
    RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0);
    buffer = 0;
    isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    buffer += 1;

    currentDegree = RobotContainer.m_chassisSubsystem.gyro.getAngle();

    if(goalDegree > 0){
      if(buffer > 20){
        RobotContainer.m_chassisSubsystem.spinToPoint(-speed, 0, 0);
      }else{
        RobotContainer.m_chassisSubsystem.spinToPoint(-0.01, 0, 0);
      }
    
    } else {
      if(buffer > 20){
        RobotContainer.m_chassisSubsystem.spinToPoint(speed, 0, 0);
      }else{
        RobotContainer.m_chassisSubsystem.spinToPoint(0.01, 0, 0);
      }

      if (currentDegree != goalDegree) {
        RobotContainer.m_chassisSubsystem.spinToPoint(0, 0, 0);
      } else {
        isFinished = true;
      }
    }
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
      isFinished = false;
      return true;
    }
    return false;
  }
}

// Below is the code for AutoChassisSpinCommand.

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;

// public class AutoChassisSpinCommand extends CommandBase {

//   double speed = 0;
//   double distance = 1;
//   double travelDistance = 0;
//   boolean isInit = false;
//   boolean isFinished = false; // boolean used to end the command. is necessary since the isFinished function
//   // is called immediatly.

//   int buffer = 0;

//   double goalDegree = 0;
//   double currentDegree = 0;
 
//   /**
//    * 
//    * @param m_degree   What direction you want to go in degrees
//    * @param m_speed    How fast you want to move in percent
//    * 
//    *                   Makes the chassis move in a desired direction at a desired
//    *                   speed.
//    */
//   public AutoChassisSpinCommand(double m_goalDegree, double m_speed) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(RobotContainer.m_chassisSubsystem);

//     goalDegree = m_goalDegree;
//     speed = m_speed * 0.01;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     RobotContainer.m_chassisSubsystem.zeroMotors();
//     buffer = 0;
//     isFinished = false;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     buffer += 1;

//     currentDegree = RobotContainer.m_chassisSubsystem.gyro.getAngle();

//     if(goalDegree > 0){
//       if(buffer > 20){
//         RobotContainer.m_chassisSubsystem.driveAuton(0, 0, -speed);
//       }else{
//         RobotContainer.m_chassisSubsystem.driveAuton(0, 0, -0.01);
      
//       }
    
//     } else {
//       if(buffer > 20){
//         RobotContainer.m_chassisSubsystem.driveAuton(0, 0, speed);
//       }else{
//         RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0.01);
//       }
//     }
   


//     if (goalDegree != 0) {
//       if ((goalDegree > 0 && currentDegree > goalDegree) || (goalDegree < 0 && currentDegree < goalDegree)) {
//         RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0);
//         isFinished = true;
//       } else {
//         isFinished = false;
//       }
//     } else {
//       RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0);
//       isFinished = true;
//     }



//     if (currentDegree != goalDegree) {
//       RobotContainer.m_chassisSubsystem.driveAuton(0, 0, 0.20);
//     } else {
//       isFinished = true;
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return isFinished;
//   }
// }
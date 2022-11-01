// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.RobotContainer;
// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class MoveClimberPnumaticsCommand extends InstantCommand {

//   DoubleSolenoid.Value direction;

//   public MoveClimberPnumaticsCommand(DoubleSolenoid.Value direction) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(RobotContainer.m_climbingSubsystem);
//     this.direction = direction;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if(direction == DoubleSolenoid.Value.kForward){
//       RobotContainer.m_climbingSubsystem.FirePiston(true);
//     }else{
//       RobotContainer.m_climbingSubsystem.FirePiston(false);
//     }
//   }
// }

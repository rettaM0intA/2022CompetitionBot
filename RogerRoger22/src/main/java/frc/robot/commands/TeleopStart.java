// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.ActiveMode;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.chassisSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TeleopStart extends InstantCommand {
//   public TeleopStart() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     activeMode = ActiveMode.teleop;

//     RobotContainer.m_chassisSubsystem = new chassisSubsystem();
//     RobotContainer.m_chassisDefaultCommand = new ChassisDefaultCommand();
    
//     RobotContainer.m_chassisSubsystem.setDefaultCommand(RobotContainer.m_chassisDefaultCommand);
//   }
// }

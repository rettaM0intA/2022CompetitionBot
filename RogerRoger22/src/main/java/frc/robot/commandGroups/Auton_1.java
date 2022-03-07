// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoChassisSpinPID;
import frc.robot.commands.ResetGyroCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auton_1 extends SequentialCommandGroup {
  /** Creates a new Auton_1. */
  public Auton_1() {   
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetGyroCommand());
    //addCommands(new AutoChassisMoveCommand(0, 50, -190));
    addCommands(new AutoChassisSpinPID(90, 50));
    addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisSpinPID(0, -50));
    addCommands(new ResetGyroCommand());
    // addCommands(new AutoChassisSpinCommand(180, 50));
  }
}
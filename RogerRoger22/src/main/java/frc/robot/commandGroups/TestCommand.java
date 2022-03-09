// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoChassisMoveCommand;
import frc.robot.commands.AutoChassisMovePid;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.IntakeDirectionChangeCommand;
import frc.robot.commands.ResetGyroCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestCommand extends SequentialCommandGroup {
  /**
   * Creates a new TestCommand.  This is meant to test commands for Autonomous mode.  Do not use this as an actual CommandGroup for competition.
   */
  public TestCommand() {
    // Add your commands in the addCommands() call, e.g.

    // addCommands(new ResetGyroCommand());
    // addCommands(new AutoChassisMoveCommand(0, 50, 2));
    // addCommands(new ResetGyroCommand());
    // addCommands(new AutoChassisMoveCommand(0, 25, 1));
    // addCommands(new ResetGyroCommand());
    // addCommands(new AutoChassisMoveCommand(0, -50, -2));
    // addCommands(new ResetGyroCommand());
    // addCommands(new AutoChassisMoveCommand(0, -25, -1));
    // addCommands(new ResetGyroCommand());
    // addCommands(new IntakeDirectionChangeCommand(), new IntakeDirectionChangeCommand());
    // addCommands(new AutoIntakeCommand(true, 3));
    // addCommands(new AutoIntakeCommand(true, 3));
    addCommands(new AutoChassisMovePid(0, -50, -3));

  }
}

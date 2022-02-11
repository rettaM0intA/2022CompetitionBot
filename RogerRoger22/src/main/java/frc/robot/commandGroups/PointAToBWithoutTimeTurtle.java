// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoChassisMovePid;
import frc.robot.commands.ResetGyroCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PointAToBWithoutTimeTurtle extends SequentialCommandGroup {
  /** Creates a new PointAToBWithoutTimeTurtle. */
  public PointAToBWithoutTimeTurtle() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
   addCommands(new AutoChassisMovePid(0, 25, 3, 3, 3, 3));
   addCommands(new ResetGyroCommand());
   addCommands(new AutoChassisMovePid(0, -25, -3, -3, -3, -3));
  }
}
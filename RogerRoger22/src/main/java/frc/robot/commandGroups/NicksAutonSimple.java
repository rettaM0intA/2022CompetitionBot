// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoChassisMovePid;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetWheelPositionCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NicksAutonSimple extends SequentialCommandGroup {
  /** Creates a new NicksAutonSimple. */
  public NicksAutonSimple() {
    addCommands(new ResetGyroCommand(), new ResetWheelPositionCommand());
    // addCommands(new AutoChassisMovePid(0, -35, -3));
    //addCommands(new ParallelCommandGroup(new IntakeSpinCommand(true, 5.0), new ResetWheelPositionCommand()));
    
    //Spit first ball
    addCommands(new AutoIntakeCommand(false, 1));
    //Move backwards
    addCommands(new AutoChassisMovePid(0, -30, 2));
  }
}

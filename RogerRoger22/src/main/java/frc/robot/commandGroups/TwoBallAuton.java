// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoChassisMovePid;
import frc.robot.commands.AutoChassisSpinPID;
import frc.robot.commands.IntakeMoverMoveCommand;
import frc.robot.commands.ResetGyroCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuton extends SequentialCommandGroup {
  /** Creates a new TwoBallAutonBlue. */
  public TwoBallAuton() {

//TODO test

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetGyroCommand());
    //Put intake down
    //addCommands(new Move(false, speedHere));
    //addCommands(new ResetGyroCommand());
    //Go to ball closest to white tape and turn to other ball
    addCommands(new AutoChassisMovePid(0, 45, 4.5));
    addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisSpinPID(70, 25));
    addCommands(new ResetGyroCommand());
    //Move to other ball, then turn to main hub thingamabob
    addCommands(new AutoChassisMovePid(0, 30, 6));
    addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisSpinPID(130, 25));
    addCommands(new ResetGyroCommand());
    //Move to main hub thingamabob
    addCommands(new AutoChassisMovePid(0, 30, 5));
    addCommands(new ResetGyroCommand());
  }
}

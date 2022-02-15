// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoChassisMovePid;
import frc.robot.commands.ResetGyroCommand;

public class AssignmentCommand extends SequentialCommandGroup {
  /** Add your docs here. */
  public AssignmentCommand() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
    addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisMovePid(0, 50, -3, -3, -3, -3));
    addCommands(new ResetGyroCommand());
    // addCommands(new AutoChassisSpinCommand(180, 50));
    // addCommands(new ResetGyroCommand());
    // addCommands(new AutoChassisSpinCommand(-180, 50));
    // addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisMovePid(0, 50, 3, 3, 3, 3));
    addCommands(new ResetGyroCommand());

    //Start at tower base, move 15 feet away in straight line, turn around 180 deg., turn around 180 deg. again, move back to tower.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}

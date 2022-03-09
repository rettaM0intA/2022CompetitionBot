// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.AutoChassisSpinPID;

public class TurnNDegreesImproved extends SequentialCommandGroup {
  /** Add your docs here. */
  public TurnNDegreesImproved() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // The goal of this is to make the robot turn 90 degrees at 30% power.
    addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisSpinPID(90, 30));
    addCommands(new ResetGyroCommand());

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

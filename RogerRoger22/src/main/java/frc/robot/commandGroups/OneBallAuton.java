// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.AutoChassisMovePid;
import frc.robot.commands.AutoChassisSpinPID;
// import frc.robot.commands.AutoIntakeCommand;

public class OneBallAuton extends SequentialCommandGroup {
  /** Add your docs here. */
  public OneBallAuton() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    addCommands(new ResetGyroCommand());
    // Robot shoots ball held at the beginning.
    // addCommands(new AutoIntakeCommand(false, 1));
    // Robot moves backward out to ball.
    addCommands(new AutoChassisMovePid(0, -30, -3));
    addCommands(new ResetGyroCommand());
    // Robot rotates 180 degrees.
    addCommands(new AutoChassisSpinPID(180, 25));
    addCommands(new ResetGyroCommand());

    // Intake is lowered.
    //addCommands(new Move(false, speedHere));

    // Intake starts running; must not stop moving. Add command.
    // Need to add a "move forward" command here.
    // Intake captures ball.
    // Intake stops running.

    // Intake is raised.
    //addCommands(new Move(true, speedHere));

    // Robot turns 180 degrees.
    addCommands(new AutoChassisSpinPID(180, 25));
    addCommands(new ResetGyroCommand());
    // Robot moves back to the goal.
    addCommands(new AutoChassisMovePid(0, 30, 3));
    // Ball is deposited.
    // addCommands(new AutoIntakeCommand(false, 1));
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

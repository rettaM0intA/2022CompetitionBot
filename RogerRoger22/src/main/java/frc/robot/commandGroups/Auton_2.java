// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoChassisMoveCommand;
import frc.robot.commands.AutoChassisMovePid;
import frc.robot.commands.AutoChassisSpinPID;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.IntakeDirectionChangeCommand;
import frc.robot.commands.IntakeSpinCommand;
import frc.robot.commands.ResetGyroCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auton_2 extends SequentialCommandGroup {
  /** Creates a new Auton_2. */
  public Auton_2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //Resets Gyro
    addCommands(new ResetGyroCommand());
    //Spit first ball
    addCommands(new AutoIntakeCommand(false, 1));
    //Move backwards
    addCommands(new AutoChassisMovePid(0, -55, 3.8));
    //Turn Around
    // addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisSpinPID(146, 25, true));
    //Drop down the intake
    addCommands(new IntakeDirectionChangeCommand());
    addCommands(new ResetGyroCommand());
    //intake on and move forward
    addCommands(new ParallelCommandGroup(new AutoChassisMovePid(0, 35, 5), new AutoIntakeCommand(true, 1.8)));
    //Turn to fix for drift
    addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisSpinPID(10, 35, true));
    //reverse and intake up
    addCommands(new ParallelCommandGroup(new AutoChassisMovePid(0, -35, 7), new IntakeDirectionChangeCommand()));
    //turn to face goal
    addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisSpinPID(160, 25, true));
    //FIRE!
    addCommands(new AutoChassisMovePid(0, 0, 0), new AutoChassisMovePid(0, 30, 3.3));
    addCommands(new AutoIntakeCommand(false, 1));
    
  }
}

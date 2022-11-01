// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDriveStraight;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoSpin;
import frc.robot.commands.IntakeDirectionChangeCommand;
// import frc.robot.commands.AutoChassisMovePid;
// import frc.robot.commands.AutoChassisSpinPID;
// import frc.robot.commands.AutoIntakeCommand;
// import frc.robot.commands.IntakeDirectionChangeCommand;
// import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.ResetGyroCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auton_3 extends SequentialCommandGroup {
  /** Auton for right side that does 2 balls */
  public Auton_3() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    //New Code
    //Resets Gyro
    addCommands(new ResetGyroCommand());
    //Move to and grab first ball
    addCommands(new IntakeDirectionChangeCommand(), new AutoSpin(0, 0, true), new ParallelCommandGroup(new AutoDriveStraight(0, .3, 35), new AutoIntakeCommand(true, 1.5, true)));
    //Move back to goal
    addCommands(new ParallelCommandGroup(new IntakeDirectionChangeCommand(), new AutoDriveStraight(0, -.7, 45)));
    //Aim and fire
    addCommands(new AutoSpin(130, 60, false), new AutoSpin(10, 40, false), new AutoDriveStraight(0.08, .3, 6), new AutoIntakeCommand(false, 1, false));

    //Old Code
    // //Resets Gyro
    // addCommands(new ResetGyroCommand());
    // //Spit first ball
    // addCommands(new AutoIntakeCommand(false, 1, true));
    // //Move backwards
    // addCommands(new AutoChassisMovePid(0, -55, 4));
    // //Turn Around
    // addCommands(new AutoChassisSpinPID(135, 45, false));
    // //Drop down the intake
    // addCommands(new IntakeDirectionChangeCommand());
    // // addCommands(new ResetGyroCommand());
    // //intake on and move forward
    // addCommands(new ParallelCommandGroup(new AutoChassisMovePid(0, 35, 1.7), new AutoIntakeCommand(true, 1.8, true)));
    // //pick up intake and move back
    // addCommands(new ParallelCommandGroup(new AutoChassisMovePid(0, 35, .5), new IntakeDirectionChangeCommand()));
    // //Move back
    // addCommands(new AutoChassisMovePid(0, -35, 1.5));

    // // addCommands(new ResetGyroCommand());
    // addCommands(new AutoChassisSpinPID(137, 25, true));
    
    // addCommands(new AutoChassisMovePid(0, 30, 4.2));

    // // addCommands(new AutoChassisSpinPID(20, 30, true));
    // //FIRE!
    // addCommands(new AutoIntakeCommand(false, 1, true));
  }
}

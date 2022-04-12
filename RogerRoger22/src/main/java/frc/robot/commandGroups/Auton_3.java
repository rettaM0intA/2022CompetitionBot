// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoChassisMovePid;
import frc.robot.commands.AutoChassisSpinPID;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.IntakeDirectionChangeCommand;
import frc.robot.commands.ResetGyroCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auton_3 extends SequentialCommandGroup {
  /** Creates a new Auton_3. */
  public Auton_3() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    //Resets Gyro
    addCommands(new ResetGyroCommand());
    //Spit first ball
    addCommands(new AutoIntakeCommand(false, 1));
    //Move backwards
    addCommands(new AutoChassisMovePid(0, -55, 4));
    //Turn Around
    addCommands(new AutoChassisSpinPID(135, 45, false));
    //Drop down the intake
    addCommands(new IntakeDirectionChangeCommand());
    // addCommands(new ResetGyroCommand());
    //intake on and move forward
    addCommands(new ParallelCommandGroup(new AutoChassisMovePid(0, 35, 1.7), new AutoIntakeCommand(true, 1.8)));
    //pick up intake and move back
    addCommands(new ParallelCommandGroup(new AutoChassisMovePid(0, 35, .5), new IntakeDirectionChangeCommand()));
    //Move back
    addCommands(new AutoChassisMovePid(0, -35, 1.5));

    // addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisSpinPID(137, 25, true));
    
    addCommands(new AutoChassisMovePid(0, 30, 4.2));

    // addCommands(new AutoChassisSpinPID(20, 30, true));
    //FIRE!
    addCommands(new AutoIntakeCommand(false, 1));
  }
}

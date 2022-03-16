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
    addCommands(new AutoChassisMovePid(0, -30, 2));
    //Turn Around
    // addCommands(new ResetGyroCommand());
    addCommands(new AutoChassisSpinPID(135, 25));
    //Drop down the intake
    addCommands(new IntakeDirectionChangeCommand());
    addCommands(new ResetGyroCommand());
    //intake on and move forward
    addCommands(new ParallelCommandGroup(new AutoChassisMovePid(0, 28, 5), new AutoIntakeCommand(true, 3)));
    //Turn back towards tower
    addCommands(new AutoChassisSpinPID(170, 25));
    addCommands(new ResetGyroCommand());
    //Move towards tower
    addCommands(new ParallelCommandGroup(new AutoChassisMovePid(0, 28, 4.5), new IntakeDirectionChangeCommand()));
    //turn towards center of tower
    addCommands(new AutoChassisSpinPID(52, 25));
    //Move towards tower
    addCommands(new AutoChassisMovePid(0, 30, 3));
    //FIRE!
    addCommands(new AutoIntakeCommand(false, 1));
  }
}

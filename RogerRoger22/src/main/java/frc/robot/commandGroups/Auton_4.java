// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.AutoChassisMovePid;
// import frc.robot.commands.AutoChassisSpinPID;
// import frc.robot.commands.AutoIntakeCommand;
// import frc.robot.commands.IntakeDirectionChangeCommand;
// import frc.robot.commands.ResetGyroCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auton_4 extends SequentialCommandGroup {
  /** Auton for right side that does 3 balls */
  public Auton_4() {
    // //Resets Gyro
    // addCommands(new ResetGyroCommand());
    // //Spit first ball
    // addCommands(new AutoIntakeCommand(false, 1, true));
    // //Move backwards //TODO fix distance
    // addCommands(new AutoChassisMovePid(0, -60, 4));
    // //Drop down the intake
    // addCommands(new IntakeDirectionChangeCommand());
    // //Turn to face first ball //TODO fix angle
    // addCommands(new AutoChassisSpinPID(135, 45, false));
    // //Intake on and move forward //TODO fix distance
    // addCommands(new ParallelCommandGroup(new AutoChassisMovePid(0, 40, 1.7), new AutoIntakeCommand(true, 1.8, true)));
    // //Move away from the wall a tiny bit //TODO make sure moving right works
    // addCommands(new AutoChassisMovePid(90, 0, 0));
    // //Turn to next ball //TODO fix angle
    // addCommands(new AutoChassisSpinPID(0, 0, true));
    // //Move to second ball with intake on //TODO fix distance
    // addCommands(new ParallelCommandGroup(new AutoChassisMovePid(0, 40, 1.7), new AutoIntakeCommand(true, 1.8, true)));
    // //Turn right to face goal //TODO fix angle
    // addCommands(new AutoChassisSpinPID(0, 0, true));
    // //Move to goal //TODO fix distance
    // addCommands(new AutoChassisMovePid(0, 0, 0));
    // //Turn to aim. //TODO fix angle
    // addCommands(new AutoChassisSpinPID(0, 0, false));
    // //FIRE
    // addCommands(new AutoIntakeCommand(false, 3, false));
  }
}

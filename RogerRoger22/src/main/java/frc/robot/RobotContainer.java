// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChassisDefaultCommand;
import frc.robot.commands.ControllerSwitchCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeMoverDefaultCommand;
import frc.robot.subsystems.IntakeMoverSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.chassisSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static chassisSubsystem m_chassisSubsystem = new chassisSubsystem();
  public static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public static IntakeMoverSubsystem m_intakeMoverSubsystem = new IntakeMoverSubsystem();

  // private static ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public static ChassisDefaultCommand m_chassisDefaultCommand = new ChassisDefaultCommand();
  public static IntakeDefaultCommand m_intakeDefaultCommand = new IntakeDefaultCommand();
  public static IntakeMoverDefaultCommand m_intakeMoverDefaultCommand = new IntakeMoverDefaultCommand();
  // public static BButtonSpinDefaultCommand bButtonSpinDefaultCommand = new BButtonSpinDefaultCommand();
  // private static WheelsFaceForwardCommand m_wheelsFaceForwardCommand = new WheelsFaceForwardCommand();

  
  public static Joystick driver = new Joystick(0);
  public static XboxController operator = new XboxController(1);

  public static ControllerInControl gamepadDriver = ControllerInControl.flightStick;

  public static boolean fullSpeed = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // m_chassisSubsystem.setDefaultCommand(new m_exampleSubsystem());\
    m_chassisSubsystem.setDefaultCommand(m_chassisDefaultCommand);
    m_intakeSubsystem.setDefaultCommand(m_intakeDefaultCommand);
    m_intakeMoverSubsystem.setDefaultCommand(m_intakeMoverDefaultCommand);
    // m_BButtonSpinSubsystem.setDefaultCommand(bButtonSpinDefaultCommand);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton padControllSwitchButton = new JoystickButton(operator, 6);
    padControllSwitchButton.whenPressed(new ControllerSwitchCommand());

    // JoystickButton testShootingButton = new JoystickButton(operator, 1);
    // testShootingButton.whenHeld(new TestShootingCommand());
    
    // JoystickButton testIntakeButton = new JoystickButton(operator, 3);
    // testIntakeButton.whenHeld(new TestShootingCommand());

    // JoystickButton bButtonSpin = new JoystickButton(operator, 2);
    // bButtonSpin.whenPressed(new BButtonSpinCommand());

    JoystickButton joyControllSwitchButton = new JoystickButton(driver, 6);
    joyControllSwitchButton.whenPressed(new ControllerSwitchCommand());
    
    if(gamepadDriver == ControllerInControl.gamepad){
      // JoystickButton padSpinButton = new JoystickButton(operator, 2);
      // padSpinButton.whenActive(new SpinCommand());
      // JoystickButton padGyroSet0Button = new JoystickButton(operator, 2);
      // padGyroSet0Button.whenPressed(new ResetGyroCommand());

    }else{
      // JoystickButton joyGyroSet0Button = new JoystickButton(driver, 9);
      // joyGyroSet0Button.whenPressed(new GyroSet0Command());

      // if(driver.getThrottle() < 0.5){
      //   fullSpeed = true;
      // }else{
      //   fullSpeed = false;
      // }
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new TestCommand();
    return Robot.m_chooser.getSelected();
  }
}

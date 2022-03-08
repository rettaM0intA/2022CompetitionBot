// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControllerInControl;
import frc.robot.RobotContainer;

public class chassisSubsystem extends SubsystemBase {

  
    /*
    *   TODO
    *   Upload to Github
    *   Add Greathouse's methods and make them work correctly
    */



  //Init Analog Encoders to help reset the wheel rotation
  public AnalogEncoder fRAnalogEncoder = new AnalogEncoder(new AnalogInput(2));
  public AnalogEncoder fLAnalogEncoder = new AnalogEncoder(new AnalogInput(0));
  public AnalogEncoder bRAnalogEncoder = new AnalogEncoder(new AnalogInput(3));
  public AnalogEncoder bLAnalogEncoder = new AnalogEncoder(new AnalogInput(1));


  //The Falcon 500s are in charge of spinning the wheels
  WPI_TalonFX fRDriveMotor = new WPI_TalonFX(6);
  WPI_TalonFX fLDriveMotor = new WPI_TalonFX(8);
  WPI_TalonFX bRDriveMotor = new WPI_TalonFX(4);
  WPI_TalonFX bLDriveMotor = new WPI_TalonFX(2);


  //The Neo550s are in charge of rotating the wheels
  public WPI_TalonFX fRrotationMotor = new WPI_TalonFX(5);
  public WPI_TalonFX fLrotationMotor = new WPI_TalonFX(7);
  public WPI_TalonFX bRrotationMotor = new WPI_TalonFX(3);
  public WPI_TalonFX bLrotationMotor = new WPI_TalonFX(1);

  // SlewRateLimiter frontLeftLimiter = new SlewRateLimiter(.72);
  // SlewRateLimiter frontRightLimiter = new SlewRateLimiter(.72);
  // SlewRateLimiter backLeftLimiter = new SlewRateLimiter(.72);
  // SlewRateLimiter backRightLimiter = new SlewRateLimiter(.72);

  ControllerInControl driver = ControllerInControl.flightStick;

  public AHRS gyro = new AHRS(I2C.Port.kOnboard);
  

  int frontLeftOnPointCount = 0;
  int frontRightOnPointCount = 0;
  int backLeftOnPointCount = 0;
  int backRightOnPointCount = 0;
  
  //The angles I want the wheels to face in
  double fRAngle = 0;
  double fLAngle = 0;
  double bRAngle = 0;
  double bLAngle = 0;
  
  //This angle is used to test functions.
  double testAngle = 0;

  double lastSpeedfL = 0;
  double lastSpeedfR = 0;
  double lastSpeedbL = 0;
  double lastSpeedbR = 0;

  PIDController fLPidController = new PIDController(0.000078, 0.000143, 0);
  PIDController fRPidController = new PIDController(0.000078, 0.000143, 0);
  PIDController bLPidController = new PIDController(0.000078, 0.000143, 0);
  PIDController bRPidController = new PIDController(0.000078, 0.000143, 0);

  PIDController fLTurnPid = new PIDController(0.105, 0, 0);
  PIDController fRTurnPid = new PIDController(0.105, 0, 0);
  PIDController bLTurnPid = new PIDController(0.105, 0, 0);
  PIDController bRTurnPid = new PIDController(0.105, 0, 0);
  
  int currentRotationFl = 1;
  int currentRotationFr = 1;
  int currentRotationBl = 1;
  int currentRotationBr = 1;

  // makes sure that the setPid method is only run once 
  boolean setPid = true;


  // Declare the swervedrive math stuff from WPI
  SwerveDriveKinematics m_kinematics;

  /** Creates a new chassisSubsystem. */
  public chassisSubsystem() {

    wheelBrakesMode();
  
  // Locations for the swerve drive modules relative to the robot center.
  Translation2d frontLeftLocation = new Translation2d(0.238125, 0.2365375);
  Translation2d frontRightLocation = new Translation2d(0.238125, -0.2365375);
  Translation2d backLeftLocation = new Translation2d(-0.238125, 0.2365375);
  Translation2d backRightLocation = new Translation2d(-0.238125, -0.2365375);

    

  fRrotationMotor.setInverted(true);
  bRrotationMotor.setInverted(true);
  fLrotationMotor.setInverted(true);
  bLrotationMotor.setInverted(true);

  if(setPid){
    SetPIDController();
  }

  m_kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  }

  
  /**
   * This is the main driving function for the AgileRunner robot.
   * @param fwd Percent forward; used to decide which direction the robot goes in with fwd.
   * @param strafe Percent strafe; used to decide which direction the robot goes in with fwd.
   * @param rotation Percent for rotating; will combine with the direction given by fwd and strafe to let the robot turn.
   */
  public void driveTeleop(double fwd, double strafe, double rotation){

    if(RobotContainer.operator.getYButton()){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }else if(RobotContainer.driver.getRawButton(9)){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }

    //The following if statements are to set controller deadzones.
    if(fwd < Constants.kDirectionalDeadzone && fwd > -Constants.kDirectionalDeadzone){
      fwd = 0;
    }
    if(strafe < Constants.kDirectionalDeadzone && strafe > -Constants.kDirectionalDeadzone){
      strafe = 0;
    }

    // convert fwd from flight stick y of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double fwd_MpS = -fwd * Constants.kChassisMaxMetersPerSec; 
    SmartDashboard.putNumber("fwd", fwd);
    SmartDashboard.putNumber("fwd_MpS", fwd_MpS);

    // convert strafe from flight stick x of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double strafe_MpS = -strafe * Constants.kChassisMaxMetersPerSec;
    SmartDashboard.putNumber("strafe", strafe);
    SmartDashboard.putNumber("strafe_MpS", strafe_MpS);

    // convert rotation from flight stick twist of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double rotation_RpS = -rotation * Constants.kChassisMaxRadiansPerSec;
    SmartDashboard.putNumber("rotation", rotation);
    SmartDashboard.putNumber("rotation_RpS", rotation_RpS);

    // Calculate the module speeds based on what the requested chassis speeds are.
    ChassisSpeeds speeds = new ChassisSpeeds(fwd_MpS,strafe_MpS,rotation_RpS);

    /**The desired field relative speed here is 2 meters per second
    // toward the opponent's alliance station wall, and 2 meters per
    // second toward the left field boundary. The desired rotation
    // is a quarter of a rotation per second counterclockwise. The current
    // robot angle is 45 degrees.
    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd_MpS, strafe_MpS, rotation_RpS, Rotation2d.fromDegrees(gyro.getAngle()));
    */
    // Get a reference to the module states for a swerve drive system
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Normalize the speeds in case one wants to be higher that the max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kChassisMaxMetersPerSec);

    // Get a reference to each module states
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    // calcAngleQuadrant(frontLeft.angle.getDegrees());
    // calcAngleQuadrant(frontRight.angle.getDegrees());
    // calcAngleQuadrant(backLeft.angle.getDegrees());
    // calcAngleQuadrant(backRight.angle.getDegrees());

    

    SwerveModuleState frontLeftOptimize = SwerveModuleState.optimize(frontLeft, new Rotation2d((fLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor * (Math.PI/180))));
    SwerveModuleState frontRightOptimize = SwerveModuleState.optimize(frontRight, new Rotation2d((fRrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor * (Math.PI/180))));
    SwerveModuleState backLeftOptimize = SwerveModuleState.optimize(backLeft, new Rotation2d((bLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor * (Math.PI/180))));
    SwerveModuleState backRightOptimize = SwerveModuleState.optimize(backRight, new Rotation2d((bRrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor * (Math.PI/180))));
  

    // Get the needed angle from the module state and convert it to the Cnts needed for the CanSparkMax PID loop
    fLAngle = (frontLeftOptimize.angle.getDegrees() / Constants.kChassisDegreetoMotor);
    fRAngle = (frontRightOptimize.angle.getDegrees() / Constants.kChassisDegreetoMotor);
    bLAngle = (backLeftOptimize.angle.getDegrees() / Constants.kChassisDegreetoMotor);
    bRAngle = (backRightOptimize.angle.getDegrees() / Constants.kChassisDegreetoMotor);


    // Get the needed speed from the module state and convert it to the -1 to 1 value needed for percent output command of the CANTalon
    double frontLeftSpeed = frontLeftOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double frontRightSpeed = -frontRightOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backLeftSpeed = backLeftOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backRightSpeed = backRightOptimize.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;

    //The goal of these four uses of rotationOverflow is to have the wheels avoid a 350+ degree rotation
    // rotationOverflow(fLrotationMotor, 0);
    // rotationOverflow(fRrotationMotor, 1);
    // rotationOverflow(bLrotationMotor, 2);
    // rotationOverflow(bRrotationMotor, 3);
    
    //these lines tell the motor controller what position to set the motor to
    // fLrotationMotor.getPIDController().setReference(fLAngle, ControlType.kPosition);
    // fRrotationMotor.getPIDController().setReference(fRAngle, ControlType.kPosition);
    // bLrotationMotor.getPIDController().setReference(bLAngle, ControlType.kPosition);
    // bRrotationMotor.getPIDController().setReference(bRAngle, ControlType.kPosition);

    // fLPidController.setSetpoint(fLAngle);
    // fRPidController.setSetpoint(fRAngle);
    // bLPidController.setSetpoint(bLAngle);
    // bRPidController.setSetpoint(bRAngle);

    fLrotationMotor.set(TalonFXControlMode.Position, (int)fLAngle);
    
    fRrotationMotor.set(TalonFXControlMode.Position, (int)fRAngle);

    bLrotationMotor.set(TalonFXControlMode.Position, (int)bLAngle);

    bRrotationMotor.set(TalonFXControlMode.Position, (int)bRAngle);

    SmartDashboard.putNumber("fRAngle", bRAngle);

    
    // fLrotationMotor.set(0.2);

    
    // Set the speed in TalonFX to a percent output.
    fLDriveMotor.set(frontLeftSpeed);
    fRDriveMotor.set(frontRightSpeed);
    bLDriveMotor.set(backLeftSpeed);
    bRDriveMotor.set(backRightSpeed);

    // fLDriveMotor.set(0);
    // fRDriveMotor.set(0);
    // bLDriveMotor.set(0);
    // bRDriveMotor.set(0);

    //fLDriveMotor.set(fLPidController.calculate(fLDriveMotor.getSelectedSensorPosition(), 1000));

  }

  
  /**
   * This is the main driving function for the AgileRunner robot.
   * @param fwd Percent forward; used to decide which direction the robot goes in with fwd.
   * @param strafe Percent strafe; used to decide which direction the robot goes in with fwd.
   * @param rotation Percent for rotating; will combine with the direction given by fwd and strafe to let the robot turn.
   */
  public void driveAuton(double fwd, double strafe, double rotation){

    if(RobotContainer.operator.getYButton()){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }else if(RobotContainer.driver.getRawButton(9)){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }

    //The following if statements are to set controller deadzones.
    if(fwd < Constants.kDirectionalDeadzone && fwd > -Constants.kDirectionalDeadzone){
      fwd = 0;
    }
    if(strafe < Constants.kDirectionalDeadzone && strafe > -Constants.kDirectionalDeadzone){
      strafe = 0;
    }

    // convert fwd from flight stick y of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double fwd_MpS = -fwd * Constants.kChassisMaxMetersPerSec; 
    SmartDashboard.putNumber("fwd", fwd);
    SmartDashboard.putNumber("fwd_MpS", fwd_MpS);

    // convert strafe from flight stick x of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double strafe_MpS = -strafe * Constants.kChassisMaxMetersPerSec;
    SmartDashboard.putNumber("strafe", strafe);
    SmartDashboard.putNumber("strafe_MpS", strafe_MpS);

    // convert rotation from flight stick twist of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double rotation_RpS = -rotation * Constants.kChassisMaxRadiansPerSec;
    SmartDashboard.putNumber("rotation", rotation);
    SmartDashboard.putNumber("rotation_RpS", rotation_RpS);

    // Calculate the module speeds based on what the requested chassis speeds are.
    ChassisSpeeds speeds = new ChassisSpeeds(fwd_MpS,strafe_MpS,rotation_RpS);

    /**The desired field relative speed here is 2 meters per second
    // toward the opponent's alliance station wall, and 2 meters per
    // second toward the left field boundary. The desired rotation
    // is a quarter of a rotation per second counterclockwise. The current
    // robot angle is 45 degrees.
    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd_MpS, strafe_MpS, rotation_RpS, Rotation2d.fromDegrees(gyro.getAngle()));
    */
    // Get a reference to the module states for a swerve drive system
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Normalize the speeds in case one wants to be higher that the max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kChassisMaxMetersPerSec);

    // Get a reference to each module states
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    
    SwerveModuleState frontLeftOptimize = SwerveModuleState.optimize(frontLeft, new Rotation2d((fLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));
    SwerveModuleState frontRightOptimize = SwerveModuleState.optimize(frontRight, new Rotation2d((fRrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));
    SwerveModuleState backLeftOptimize = SwerveModuleState.optimize(backLeft, new Rotation2d((bLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));
    SwerveModuleState backRightOptimize = SwerveModuleState.optimize(backRight, new Rotation2d((bRrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));

    // Get the needed angle from the module state and convert it to the Cnts needed for the CanSparkMAx PID loop
    fLAngle = (frontLeftOptimize.angle.getDegrees() / Constants.kChassisDegreetoMotor);
    fRAngle = (frontRightOptimize.angle.getDegrees() / Constants.kChassisDegreetoMotor);
    bLAngle = (backLeftOptimize.angle.getDegrees() / Constants.kChassisDegreetoMotor);
    bRAngle = (backRightOptimize.angle.getDegrees() / Constants.kChassisDegreetoMotor);


    // Get the needed speed from the module state and convert it to the -1 to 1 value needed for percent output command of the CANTalon
    double frontLeftSpeed = frontLeft.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double frontRightSpeed = frontRight.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backLeftSpeed = backLeft.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backRightSpeed = backRight.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;

    //The goal of these four uses of rotationOverflow is to have the wheels avoid a 350+ degree rotation
    rotationOverflow(fLrotationMotor, 0);
    rotationOverflow(fRrotationMotor, 1);
    rotationOverflow(bLrotationMotor, 2);
    rotationOverflow(bRrotationMotor, 3);
    
    //TODO Fix PID stuff

    //these lines tell the motor controller what poisition to set the motor to
    // fLrotationMotor.getPIDController().setReference(fLAngle, ControlType.kPosition);
    // fRrotationMotor.getPIDController().setReference(fRAngle, ControlType.kPosition);
    // bLrotationMotor.getPIDController().setReference(bLAngle, ControlType.kPosition);
    // bRrotationMotor.getPIDController().setReference(bRAngle, ControlType.kPosition);
    

    //WIP for replacing the avoidance method of infinite rotation with real infinite rotation.
    //Uses the turn forever method of going past full rotation.  Make sure to comment out rotationOverflow calls and the previous inputs to the rotation motors.
    // fLrotationMotor.getPIDController().setReference(TurnForever(fLrotationMotor, fLAngle), ControlType.kPosition);
    // fRrotationMotor.getPIDController().setReference(TurnForever(fRrotationMotor, fRAngle), ControlType.kPosition);
    // bLrotationMotor.getPIDController().setReference(TurnForever(bLrotationMotor, bLAngle), ControlType.kPosition);
    // bRrotationMotor.getPIDController().setReference(TurnForever(bRrotationMotor, bRAngle), ControlType.kPosition);
    
    
    
    // Set the speed in TalonFX to a percent output.

    fLrotationMotor.set(TalonFXControlMode.Position, fLAngle);
    fRrotationMotor.set(TalonFXControlMode.Position, fRAngle);
    bLrotationMotor.set(TalonFXControlMode.Position, fLAngle);
    bRrotationMotor.set(TalonFXControlMode.Position, bRAngle);

    fLDriveMotor.set((frontLeftSpeed));
    fRDriveMotor.set((frontRightSpeed));
    bLDriveMotor.set((backLeftSpeed));
    bRDriveMotor.set((backRightSpeed));


    // lastSpeedfL = frontLeftSpeed;
    // lastSpeedfR = frontRightSpeed;
    // lastSpeedbL = backLeftSpeed;
    // lastSpeedbR = backRightSpeed;

  }

  private double speedLimiter(double input){
    
    if(Math.abs(input) > 0.6){
      if(input > 0){
        return 0.6;
      }else{
        return -0.6;
      }
    }

    return input;
  }

  
  /**
   * THIS DESCRIPTION SUCKS. FIX IT. This is the main driving function for the AgileRunner robot.
   * @param fwd Percent forward.  Used to decide which direction the robot goes in with fwd
   * @param strafe Percent strafe.  Used to decide which direction the robot goes in with fwd
   * @param rotation Percent for rotating.  Will combine with the direction given by fwd and strafe to let the robot turn.
   */
  public void driveToPoint(double fwd, double strafe, double rotation, double fLgoalPosition, double fRgoalPosition, double bLgoalPosition, double bRgoalPosition){

    if(RobotContainer.operator.getYButton()){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }else if(RobotContainer.driver.getRawButton(9)){
      resetGyro();
      fRDriveMotor.setSelectedSensorPosition(0);
      fLDriveMotor.setSelectedSensorPosition(0);
      bRDriveMotor.setSelectedSensorPosition(0);
      bLDriveMotor.setSelectedSensorPosition(0);
    }

    //The following if statements are to set controller deadzones.
    if(fwd < Constants.kDirectionalDeadzone && fwd > -Constants.kDirectionalDeadzone){
      fwd = 0;
    }
    if(strafe < Constants.kDirectionalDeadzone && strafe > -Constants.kDirectionalDeadzone){
      strafe = 0;
    }

    // convert fwd from flight stick y of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double fwd_MpS = -fwd * Constants.kChassisMaxMetersPerSec;
    SmartDashboard.putNumber("fwd", fwd);
    SmartDashboard.putNumber("fwd_MpS", fwd_MpS);

    // convert strafe from flight stick x of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double strafe_MpS = -strafe * Constants.kChassisMaxMetersPerSec;
    SmartDashboard.putNumber("strafe", strafe);
    SmartDashboard.putNumber("strafe_MpS", strafe_MpS);

    // convert rotation from flight stick twist of (-1 to 1) to MetersPerSecond for ChassisSpeeds
    double rotation_RpS = -rotation * Constants.kChassisMaxRadiansPerSec;
    SmartDashboard.putNumber("rotation", rotation);
    SmartDashboard.putNumber("rotation_RpS", rotation_RpS);

    // Calculate the module speeds based on what the requested chassis speeds are.
    ChassisSpeeds speeds = new ChassisSpeeds(fwd_MpS,strafe_MpS,rotation_RpS);

    /**The desired field relative speed here is 2 meters per second
    // toward the opponent's alliance station wall, and 2 meters per
    // second toward the left field boundary. The desired rotation
    // is a quarter of a rotation per second counterclockwise. The current
    // robot angle is 45 degrees.
    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(fwd_MpS, strafe_MpS, rotation_RpS, Rotation2d.fromDegrees(gyro.getAngle()));
    */
    // Get a reference to the module states for a swerve drive system
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Normalize the speeds in case one wants to be higher that the max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kChassisMaxMetersPerSec);

    // Get a reference to each module states
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];
    
    // SwerveModuleState frontLeftOptimize = SwerveModuleState.optimize(frontLeft, new Rotation2d((fLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));
    // SwerveModuleState frontRightOptimize = SwerveModuleState.optimize(frontRight, new Rotation2d((fLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));
    // SwerveModuleState backLeftOptimize = SwerveModuleState.optimize(backLeft, new Rotation2d((fLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));
    // SwerveModuleState backRightOptimize = SwerveModuleState.optimize(backRight, new Rotation2d((fLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));

    // Get the needed angle from the module state and convert it to the Cnts needed for the CanSparkMAx PID loop
    fLAngle = (frontLeft.angle.getDegrees() / Constants.kChassisDegreetoMotor);
    fRAngle = (frontRight.angle.getDegrees() / Constants.kChassisDegreetoMotor);
    bLAngle = (backLeft.angle.getDegrees() / Constants.kChassisDegreetoMotor);
    bRAngle = (backRight.angle.getDegrees() / Constants.kChassisDegreetoMotor);


    // Get the needed speed from the module state and convert it to the -1 to 1 value needed for percent output command of the CANTalon
    double frontLeftSpeed = frontLeft.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double frontRightSpeed = frontRight.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backLeftSpeed = backLeft.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backRightSpeed = backRight.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;

    //The goal of these four uses of rotationOverflow is to have the wheels avoid a 350+ degree rotation
    
    fLrotationMotor.set(TalonFXControlMode.Position, (int)fLAngle);
    
    fRrotationMotor.set(TalonFXControlMode.Position, (int)fRAngle);

    bLrotationMotor.set(TalonFXControlMode.Position, (int)bLAngle);

    bRrotationMotor.set(TalonFXControlMode.Position, (int)bRAngle);
    
    //these lines tell the motor controller what poisition to set the motor to
    // fLrotationMotor.getPIDController().setReference(fLAngle, ControlType.kPosition);
    // fRrotationMotor.getPIDController().setReference(fRAngle, ControlType.kPosition);
    // bLrotationMotor.getPIDController().setReference(bLAngle, ControlType.kPosition);
    // bRrotationMotor.getPIDController().setReference(bRAngle, ControlType.kPosition);
    

    //WIP for replacing the avoidance method of infinite rotation with real infinite rotation.
    //Uses the turn forever method of going past full rotation.  Make sure to comment out rotationOverflow calls and the previous inputs to the rotation motors.
    // fLrotationMotor.getPIDController().setReference(TurnForever(fLrotationMotor, fLAngle), ControlType.kPosition);
    // fRrotationMotor.getPIDController().setReference(TurnForever(fRrotationMotor, fRAngle), ControlType.kPosition);
    // bLrotationMotor.getPIDController().setReference(TurnForever(bLrotationMotor, bLAngle), ControlType.kPosition);
    // bRrotationMotor.getPIDController().setReference(TurnForever(bRrotationMotor, bRAngle), ControlType.kPosition);
    
    // Set the speed in TalonFX to a percent output.
    // fLrotationMotor.set(frontLeftSpeed);
    // fRrotationMotor.set(-frontRightSpeed);
    // bLrotationMotor.set(backLeftSpeed);
    // bRrotationMotor.set(-backRightSpeed);

    
    

    // fLDriveMotor.set(toPointSpeedLimit(fLPidController.calculate(fLDriveMotor.getSelectedSensorPosition(), fLgoalPosition)));
    // fRDriveMotor.set(toPointSpeedLimit(-fRPidController.calculate(fRDriveMotor.getSelectedSensorPosition(), -fRgoalPosition)));
    // bLDriveMotor.set(toPointSpeedLimit(bLPidController.calculate(bLDriveMotor.getSelectedSensorPosition(), bLgoalPosition)));
    // bRDriveMotor.set(toPointSpeedLimit(-bRPidController.calculate(bRDriveMotor.getSelectedSensorPosition(), -bRgoalPosition))); 


    // fLPidController.calculate(fLrotationMotor.getSelectedSensorPosition(), -fLgoalPosition);
    // fRPidController.calculate(fLrotationMotor.getSelectedSensorPosition(), -fRgoalPosition);
    // bLPidController.calculate(fLrotationMotor.getSelectedSensorPosition(), -bLgoalPosition);
    // bRPidController.calculate(fLrotationMotor.getSelectedSensorPosition(), bRgoalPosition);

    fLDriveMotor.set(ControlMode.Position, fLgoalPosition);
    fRDriveMotor.set(ControlMode.Position, fRgoalPosition);
    bLDriveMotor.set(ControlMode.Position, bLgoalPosition);
    bRDriveMotor.set(ControlMode.Position, -bRgoalPosition);

    // lastSpeedfL = frontLeftSpeed;
    // lastSpeedfR = frontRightSpeed;
    // lastSpeedbL = backLeftSpeed;
    // lastSpeedbR = backRightSpeed;

  }

  /**
   * This is the main turning function for the AgileRunner robot.
   * @param rotation Percent for rotating; will combine with the direction given by fwd and strafe to let the robot turn.
   * @param fwd Percent forward; used to decide which direction the robot goes in with fwd.
   * @param strafe Percent strafe; used to decide which direction the robot goes in with fwd.
   */
  public void spinToPoint(double rotation, double fwd, double strafe){
    
    double fwd_MpS = 0; 
    SmartDashboard.putNumber("fwd", fwd);
    SmartDashboard.putNumber("fwd_MpS", fwd_MpS);

    double strafe_MpS = 0;
    SmartDashboard.putNumber("strafe", strafe);
    SmartDashboard.putNumber("strafe_MpS", strafe_MpS);

    double rotation_RpS = -rotation * Constants.kChassisMaxRadiansPerSec;
    SmartDashboard.putNumber("rotation", rotation);
    SmartDashboard.putNumber("rotation_RpS", rotation_RpS);

    ChassisSpeeds speeds = new ChassisSpeeds(fwd_MpS,strafe_MpS,rotation_RpS);

    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.kChassisMaxMetersPerSec);

    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    SwerveModuleState frontLeftOptimize = SwerveModuleState.optimize(frontLeft, new Rotation2d((fLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));
    SwerveModuleState frontRightOptimize = SwerveModuleState.optimize(frontRight, new Rotation2d((fLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));
    SwerveModuleState backLeftOptimize = SwerveModuleState.optimize(backLeft, new Rotation2d((fLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));
    SwerveModuleState backRightOptimize = SwerveModuleState.optimize(backRight, new Rotation2d((fLrotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor) * 0.0174533));

    fLAngle = (frontLeftOptimize.angle.getDegrees()) / Constants.kChassisDegreetoMotor;
    fRAngle = (frontRightOptimize.angle.getDegrees()) / Constants.kChassisDegreetoMotor;
    bLAngle = (backLeftOptimize.angle.getDegrees()) / Constants.kChassisDegreetoMotor;
    bRAngle = (backRightOptimize.angle.getDegrees()) / Constants.kChassisDegreetoMotor;

    double frontLeftSpeed = frontLeft.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double frontRightSpeed = frontRight.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backLeftSpeed = backLeft.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;
    double backRightSpeed = backRight.speedMetersPerSecond / Constants.kChassisMotorSpeedLower;

    rotationOverflow(fLrotationMotor, 0);
    rotationOverflow(fRrotationMotor, 1);
    rotationOverflow(bLrotationMotor, 2);
    rotationOverflow(bRrotationMotor, 3);

    fLDriveMotor.set(frontLeftSpeed);
    fRDriveMotor.set(-frontRightSpeed);
    bLDriveMotor.set(backLeftSpeed);
    bRDriveMotor.set(-backRightSpeed);
    
  }
  
  private double toPointSpeedLimit(double attemptSpeed){
    
    if(Math.abs(attemptSpeed) > 0.7){
      if(attemptSpeed > 0){
        return 0.65;
      }else{
        return -0.65;
      }
    }

    return attemptSpeed;
  }

//   private int calcQuad(double _desiredAngle) {
//     int desiredQuad = 1;
//     if (_desiredAngle >= 0 && _desiredAngle < 90) { desiredQuad = 1; } else 
//     if (_desiredAngle >= 90 && _desiredAngle < 180) { desiredQuad = 2; } else 
//     if (_desiredAngle < 0 && _desiredAngle > -90) { desiredQuad = -1; } else 
//     if (_desiredAngle < -90 && _desiredAngle > -180) { desiredQuad = -2; }
//     return desiredQuad;
// }

// private int calcAngleQuadrant(double _desiredAngle, int _previousDesiredQuad) {
    // int desiredQuad = calcQuad(_desiredAngle);
    // int desiredQuadDirectionIndex = Math.abs(desiredQuad - 1);
    // int previousDesiredQuadDirectionIndex = Math.abs(_previousDesiredQuad - 1);
    // int desiredQuadDirection = m_desiredQuadDirIndexArray[desiredQuadDirectionIndex][previousDesiredQuadDirectionIndex];
    // int virtualQuadIndex = m_virtualQuad < 0 ? Math.abs(m_virtualQuad - 3) : Math.abs(m_virtualQuad - 1);
    // m_virtualQuad = m_virtualQuadArray[desiredQuadDirection][virtualQuadIndex];
    // m_previousDesiredQuad = desiredQuad;
    // return m_virtualQuad;
// }

  /**
   * A function made to avoid going to 0 or 360 degrees in rotation.
   * @param rotationMotor Used to check a motor's position to avoid doing a full rotation.
   * @param angleNumber Input an integer based on which motor is being used.  FL = 0, FR = 1, BL = 2, BR = 3
   */
  public void rotationOverflow(WPI_TalonFX rotationMotor, int angleNumber){
    double currentRotation = rotationMotor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor;
    double goalAngle = 0;   //used as a reference of the Rotation Motor's goal rotation in degrees.
    
    if(angleNumber == 0){
      goalAngle = fLAngle * Constants.kChassisDegreetoMotor;
    }else if(angleNumber == 1){
      goalAngle = fRAngle * Constants.kChassisDegreetoMotor;
    }else if(angleNumber == 2){
      goalAngle = bLAngle * Constants.kChassisDegreetoMotor;
    }else if(angleNumber == 3){
      goalAngle = bRAngle * Constants.kChassisDegreetoMotor;
    }

    //Checks to see if the goal rotation is near +-180 degrees.  If it is the goal rotation is flipped to avoid having the wheel do a 360
    if(goalAngle > 170 || goalAngle < -170){
      if(currentRotation > 0){
        goalAngle = goalAngle - 180;
      }else{
        goalAngle = goalAngle + 180;
      }
    }

    if(angleNumber == 0){
      fLAngle = goalAngle / Constants.kChassisDegreetoMotor;
    }else if(angleNumber == 1){
      fRAngle = goalAngle / Constants.kChassisDegreetoMotor;
    }else if(angleNumber == 2){
      bLAngle = goalAngle / Constants.kChassisDegreetoMotor;
    }else if(angleNumber == 3){
      bRAngle = goalAngle / Constants.kChassisDegreetoMotor;
    }
    
  }


  /**
   * Attempt at fixing the inablity to go past a full rotoation with the wheel
   * @param motor The rotation motor.
   * @param goalAngle The angle that the rotoation moter should be at.
   * @return Returns the new goal angle that will allow the rotation motors to go past a full rotation.
   */
  // public double TurnForever(CANSparkMax motor, double goalAngle){

  //   double convertedTo180s = motor.getEncoder().getPosition() * Constants.kChassisDegreetoMotor;

  //   SmartDashboard.putNumber("LookATME!", goalAngle);
    
  //   if((goalAngle < 0) && goalAngle + (currentRotation * 360) - 180 > convertedTo180s){
  //     currentRotation = currentRotation - 1;
  //   }else if((goalAngle > 0) && goalAngle + (currentRotation * 360) + 180 < convertedTo180s){
  //       currentRotation = currentRotation + 1;
  //   }
    
  //   goalAngle = (currentRotation * 360) + goalAngle;
    


  //   return goalAngle / Constants.kChassisDegreetoMotor;

  // }

  public double TurnForever(WPI_TalonFX motor, double angleNumber){

    double motorAngle = motor.getSelectedSensorPosition() * Constants.kChassisDegreetoMotor;
    double goalPosition = 0;
    double goalAngle = 0;

    if(angleNumber == 0){
      goalAngle = fLAngle * Constants.kChassisDegreetoMotor;
    }else if(angleNumber == 1){
      goalAngle = fRAngle * Constants.kChassisDegreetoMotor;
    }else if(angleNumber == 2){
      goalAngle = bLAngle * Constants.kChassisDegreetoMotor;
    }else if(angleNumber == 3){
      goalAngle = bRAngle * Constants.kChassisDegreetoMotor;
    }

    return goalPosition;
  }


  

  /**
   * Makes the drive motors go in coast mode
   */
  public void wheelBrakesMode(){
    fLDriveMotor.setNeutralMode(NeutralMode.Coast);
    fRDriveMotor.setNeutralMode(NeutralMode.Coast);
    bLDriveMotor.setNeutralMode(NeutralMode.Coast);
    bRDriveMotor.setNeutralMode(NeutralMode.Coast);

    fLrotationMotor.setNeutralMode(NeutralMode.Brake);
    bLrotationMotor.setNeutralMode(NeutralMode.Brake);
    fRrotationMotor.setNeutralMode(NeutralMode.Brake);
    bRrotationMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * @return Average position of all 4 drive motors
   */
  public double wheelMotorCountAverage(){
    return (
    -fRDriveMotor.getSelectedSensorPosition() +
     fLDriveMotor.getSelectedSensorPosition() +
    -bRDriveMotor.getSelectedSensorPosition() +
     bLDriveMotor.getSelectedSensorPosition())/ 4;
  }


  
  /**
   * Get the chassis angle
   */
  public double getChassisAngle(){
    return gyro.getAngle();
  }


  
  // Creates the PID controllers for all 4 rotation motors.  Should only ever be called once
  public void SetPIDController(){
    fLrotationMotor.setSelectedSensorPosition(0);
    fRrotationMotor.setSelectedSensorPosition(0);
    bLrotationMotor.setSelectedSensorPosition(0);
    bRrotationMotor.setSelectedSensorPosition(0);

    fLrotationMotor.config_kP(0, 0.105);
    fLrotationMotor.config_kI(0, 0);
    fLrotationMotor.config_kD(0, 0);
    fLrotationMotor.config_kF(0, 0);

    fRrotationMotor.config_kP(0, 0.105);
    fRrotationMotor.config_kI(0, 0);
    fRrotationMotor.config_kD(0, 0);
    fRrotationMotor.config_kF(0, 0);

    bLrotationMotor.config_kP(0, 0.105);
    bLrotationMotor.config_kI(0, 0);
    bLrotationMotor.config_kD(0, 0);
    bLrotationMotor.config_kF(0, 0);

    bRrotationMotor.config_kP(0, 0.105);
    bRrotationMotor.config_kI(0, 0);
    bRrotationMotor.config_kD(0, 0);
    bRrotationMotor.config_kF(0, 0);

    fLDriveMotor.config_kP(0, 0.025);
    fLDriveMotor.config_kI(0, 0);
    fLDriveMotor.config_kD(0, 0);
    fLDriveMotor.config_kF(0, 0);
    
    fRDriveMotor.config_kP(0, 0.025);
    fRDriveMotor.config_kI(0, 0);
    fRDriveMotor.config_kD(0, 0);
    fRDriveMotor.config_kF(0, 0);
    
    bLDriveMotor.config_kP(0, 0.025);
    bLDriveMotor.config_kI(0, 0);
    bLDriveMotor.config_kD(0, 0);
    bLDriveMotor.config_kF(0, 0);
    
    bRDriveMotor.config_kP(0, 0.025);
    bRDriveMotor.config_kI(0, 0);
    bRDriveMotor.config_kD(0, 0);
    bRDriveMotor.config_kF(0, 0);
    
    fLDriveMotor.configClosedloopRamp(0.01);
    fRDriveMotor.configClosedloopRamp(0.01);
    bLDriveMotor.configClosedloopRamp(0.01);
    bRDriveMotor.configClosedloopRamp(0.01);
    
    

    setPid = false; //Since the if statement that calls this function requires this boolean to be true, this prevents it from being rerun
  }

  public boolean checkPIDlocation(){

    if(fRPidController.atSetpoint()){
      frontLeftOnPointCount += 1;
    }else{
      frontLeftOnPointCount = 0;
    }
    SmartDashboard.putNumber("frontleftPID", frontLeftOnPointCount);
    
    // if(fRDriveMotor.)

    if(fRPidController.atSetpoint()){
      frontRightOnPointCount += 1;
    }else{
      frontRightOnPointCount = 0;
    }
    SmartDashboard.putNumber("frontRightPID", frontRightOnPointCount);
    
    if(bLPidController.atSetpoint()){
      backLeftOnPointCount += 1;
    }else{
      backLeftOnPointCount = 0;
    }
    SmartDashboard.putNumber("backleftPID", backLeftOnPointCount);
    
    if(bRPidController.atSetpoint()){
      backRightOnPointCount += 1;
    }else{
      backRightOnPointCount = 0;
    }
    SmartDashboard.putNumber("backrightPID", backRightOnPointCount);

    if(frontLeftOnPointCount >= 5 && frontRightOnPointCount >= 5 && backLeftOnPointCount >= 5 && backRightOnPointCount >= 5){
 
      frontLeftOnPointCount = 0;
      frontRightOnPointCount = 0;
      backLeftOnPointCount = 0;
      backRightOnPointCount = 0;
     return true;
    }

    return false;
  }



  public void disablePids(){
    fLPidController.close();
    fRPidController.close();
    bLPidController.close();
    bRPidController.close();
  }

  
  /**
   * Zeros all 4 drive motors 
   */
  public void zeroMotors(){
    fRDriveMotor.setSelectedSensorPosition(0);
    fLDriveMotor.setSelectedSensorPosition(0);
    bRDriveMotor.setSelectedSensorPosition(0);
    bLDriveMotor.setSelectedSensorPosition(0);
  }

  
  /**
   * Resets the gyro
   */
  public void resetGyro(){
    gyro.reset();
  }



  /**
   * Calls all smartdashboard data placements
   */
  public void smartDashboardCall(){

    // SmartDashboard.putNumber("FrontRightEncoder", fRAnalogEncoder.get());
    // SmartDashboard.putNumber("FrontLeftEncoder", fLAnalogEncoder.get());
    // SmartDashboard.putNumber("BackRightEncoder", bRAnalogEncoder.get());
    // SmartDashboard.putNumber("BackLeftEncoder", bLAnalogEncoder.get());
    
 
    SmartDashboard.putNumber("Gyro position", gyro.getAngle());
     SmartDashboard.putData(gyro);
     SmartDashboard.putBoolean("Gyro connected", gyro.isConnected());
 
    //  SmartDashboard.putNumber("fR Rotation", fRrotationMotor.getSelectedSensorPosition() / Constants.kChassisFalconToWheelRatio * 360);
     SmartDashboard.putNumber("fL Rotation", fLrotationMotor.getSelectedSensorPosition() / Constants.kChassisFalconToWheelRatio * 360);
    //  SmartDashboard.putNumber("bR Rotation", bRrotationMotor.getSelectedSensorPosition() / Constants.kChassisFalconToWheelRatio * 360);
     SmartDashboard.putNumber("bL Rotation", bLrotationMotor.getSelectedSensorPosition());
 
     SmartDashboard.putNumber("FrontLeftEncoder", fLAnalogEncoder.get());   
     SmartDashboard.putNumber("FrontRightEncoder", fRAnalogEncoder.get());
     SmartDashboard.putNumber("BackLeftEncoder", bLAnalogEncoder.get());
     SmartDashboard.putNumber("BackRightEncoder", bRAnalogEncoder.get());
 
     SmartDashboard.putNumber("rotations traveled", (-fRDriveMotor.getSelectedSensorPosition() + fLDriveMotor.getSelectedSensorPosition() - bRDriveMotor.getSelectedSensorPosition() + bLDriveMotor.getSelectedSensorPosition()) / 4);
     SmartDashboard.putNumber("Fr", fRDriveMotor.getSelectedSensorPosition());
     SmartDashboard.putNumber("Fl", fLDriveMotor.getSelectedSensorPosition());
     SmartDashboard.putNumber("br", bRDriveMotor.getSelectedSensorPosition());
     SmartDashboard.putNumber("bl", bLDriveMotor.getSelectedSensorPosition());
    //  SmartDashboard.putNumber("Wheel power", fRDriveMotor.get());

  
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run.  Even when disabled.
    smartDashboardCall(); 
  }

  
}
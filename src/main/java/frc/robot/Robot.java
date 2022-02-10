// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  
  WPI_TalonFX FLeft = new WPI_TalonFX(0);
  WPI_TalonFX BLeft = new WPI_TalonFX(1);
  WPI_TalonFX FRight = new WPI_TalonFX(2);
  WPI_TalonFX BRight = new WPI_TalonFX(3);
  MecanumDrive mecanum = new MecanumDrive(FLeft, BLeft, FRight, BRight);

  Joystick joy = new Joystick(0);
  //mechanisms' motors
  //WPI_TalonSRX intakeLeft = new WPI_TalonSRX(0);
  //WPI_TalonSRX intakeRight = new WPI_TalonSRX(0);
 // WPI_TalonSRX intakeBelt = new WPI_TalonSRX(0);
  //WPI_TalonSRX shootingBelt = new WPI_TalonSRX(0);
  //CANSparkMax climbMotor = new CANSparkMax(0, MotorType.kBrushless);

  //Encoder Converter
  double distEncode = 26465/18; //26,465 units per 18 inches
  //Encoder Double Values
  double bLeftPos;
  double bRightPos;
  double fLeftPos;
  double fRightPos;
  double navXV;

  //sensors
  AHRS navX = new AHRS(Port.kMXP);

  // X52 button mapping 
  int axisX = 0;// axis 1
  int axisY = 1;// axis 2
  int axisZ = 2;// axis 3
  int rotX = 3; // axis 4
  int rotY = 4; // axis 5
  int rotZ = 5; // axis 6
  int slider = 6;// axis 7
  int trigger = 1;
  int Fire = 2;
  int A = 3;
  int B = 4;
  int C = 5;
  int pink = 6;
  int D = 7;
  int E = 8;
  int T1 = 9;
  int T2 = 10;
  int T3 = 11;
  int T4 = 12;
  int T5 = 13;
  int T6 = 14;
  int povUp = 16;
  int povRight = 17;
  int povDown = 18;
  int povLeft = 19;
  int thumbUp = 20;
  int thumbRight = 21;
  int thumbDown = 22;
  int thumbLeft = 23;
  int ModeG = 24;
  int ModeO = 25;
  int ModeR = 26;
  int i = 30;
  int button = 31;
  int scroll = 32;

  //autonomous
  int x = 1;


  //limelight:
  // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // NetworkTableEntry tx = table.getEntry("tx");
  // NetworkTableEntry ty = table.getEntry("ty");
  // NetworkTableEntry ta = table.getEntry("ta");
  // double x = tx.getDouble(0.0);
  // double y = ty.getDouble(0.0);
  // double area = ta.getDouble(0.0);

  // private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_robotContainer = new RobotContainer();


    SmartDashboard.putData("Auto choices", m_chooser);

    //climber.configSelectedFeedbackSensor();
    BRight.setSafetyEnabled(false);
    FRight.setSafetyEnabled(false);
    BLeft.setSafetyEnabled(false);
    FLeft.setSafetyEnabled(false); 
    mecanum.setSafetyEnabled(false);
    //reverse direction of the left side motors
    BLeft.setInverted(true);
    FLeft.setInverted(true);


  //  SmartDashboard.putNumber("Limelight", x);
    
    

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  //  SmartDashboard.putNumber("Limelight", x);
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    bLeftPos = BLeft.getSelectedSensorPosition(TalonFXFeedbackDevice.IntegratedSensor.value);
    bRightPos = BRight.getSelectedSensorPosition(TalonFXFeedbackDevice.IntegratedSensor.value);
    fLeftPos = FLeft.getSelectedSensorPosition(TalonFXFeedbackDevice.IntegratedSensor.value);
    fRightPos = FRight.getSelectedSensorPosition(TalonFXFeedbackDevice.IntegratedSensor.value);
    navXV = navX.getAngle();

    SmartDashboard.putNumber("Back Left Position", bLeftPos);
    SmartDashboard.putNumber("Back Right Position", bRightPos);
    SmartDashboard.putNumber("Front Left Position", fLeftPos);
    SmartDashboard.putNumber("Front Right Position", fRightPos);
    SmartDashboard.putNumber("navX:",navXV);
    SmartDashboard.putNumber("x:", x);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    BLeft.setSelectedSensorPosition(0);
    BRight.setSelectedSensorPosition(0);
    FLeft.setSelectedSensorPosition(0);
    FRight.setSelectedSensorPosition(0);

    x = 1;
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }

  }


  

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //goes backwards
    if(FLeft.getSelectedSensorPosition() < 24*distEncode && x == 1){
      mecanum.driveCartesian(0.25, 0, 0);
    }else if(x == 1){
      x = 2;
    }
    //turn right
    if(navX.getAngle() < 80 && x == 2){
      mecanum.driveCartesian(0, 0, -0.25);
    }else if(x == 2){
      mecanum.driveCartesian(0, 0, 0);
      FLeft.setSelectedSensorPosition(0, 0, 0);
      x = 3;
    }

    //goes backwards
    if(FLeft.getSelectedSensorPosition() < 24*distEncode && x == 3){
      mecanum.driveCartesian(0.25, 0, 0);
    }else if(x == 3){
      navX.reset();
      x = 4;
    }
    //turns right
    if(navX.getAngle() < 80 && x == 4){
      mecanum.driveCartesian(0, 0, -0.25);
    }else if(x == 4){
      mecanum.driveCartesian(0, 0, 0);
      FLeft.setSelectedSensorPosition(0, 0, 0);
      x = 5;
    }



    //goes backwards
    if(FLeft.getSelectedSensorPosition() < 24*distEncode && x == 5){
      mecanum.driveCartesian(0.25, 0, 0);
    }else if(x == 5){
      navX.reset();
      x = 6;
    }
    //turn right
    if(navX.getAngle() < 80 && x == 6){
      mecanum.driveCartesian(0, 0, -0.25);
    }else if(x == 6){
      mecanum.driveCartesian(0, 0, 0);
      FLeft.setSelectedSensorPosition(0, 0, 0);
      x = 7;
    }

    //goes backwards
    if(FLeft.getSelectedSensorPosition() < 24*distEncode && x == 7){
      mecanum.driveCartesian(0.25, 0, 0);
    }else if(x == 7){
      navX.reset();
      x = 8;
    }
    //turns right
    if(navX.getAngle() < 80 && x == 8){
      mecanum.driveCartesian(0, 0, -0.25);
    }else if(x == 8){
      mecanum.driveCartesian(0, 0, 0);
      FLeft.setSelectedSensorPosition(0, 0, 0);
      x = 9;
    }


  /* if(joy.getRawButton(Fire)){
    BLeft.setSelectedSensorPosition(0);
    BRight.setSelectedSensorPosition(bRightPos, 0, 0);
    FLeft.setSelectedSensorPosition(0, TalonFXFeedbackDevice.IntegratedSensor.value, 0);
    FRight.setSelectedSensorPosition(0.0, TalonFXFeedbackDevice.IntegratedSensor.value, 0);
  }*/
SmartDashboard.putNumber("Match Position", (18*4*distEncode));
if(joy.getRawButton(A)){
  if(fRightPos < (18*4*distEncode))
    FRight.set(0.25);
  else  
    FRight.set(0.0);
}


}
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(joy.getRawButton(Fire)){
      BLeft.setSelectedSensorPosition(0);
      BRight.setSelectedSensorPosition(0);
      FLeft.setSelectedSensorPosition(0);
      FRight.setSelectedSensorPosition(0);

      navX.reset();
    }

    mecanum.driveCartesian(joy.getRawAxis(axisY), joy.getRawAxis(axisX), joy.getRawAxis(rotZ));
    //Intake();
  }
  public void Intake(){
    /*if(joy.getRawButton(Fire)){
      mecanum.driveCartesian(0, 50, 0);
    }
    else{
      mecanum.driveCartesian(0, 0, 0);
    }*/
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

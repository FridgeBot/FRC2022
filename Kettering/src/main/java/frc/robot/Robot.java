// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
//import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
import edu.wpi.first.wpilibj.Solenoid;


public class Robot extends TimedRobot {
  

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //Dashboard options for autonomous 
  private static final String Test = "Test";
  private static final String OneMove = "OneMove";
  private static final String None = "None";
  private static final String twoBalls = "twoBalls";
  private static final String threeBalls = "threeBalls";

  private String m_autoSelected;

  Timer timer = new Timer();
  //drive motors
  WPI_TalonFX FLeft = new WPI_TalonFX(0);
  WPI_TalonFX BLeft = new WPI_TalonFX(1);
  WPI_TalonFX FRight = new WPI_TalonFX(2);
  WPI_TalonFX BRight = new WPI_TalonFX(3);
  MecanumDrive mecanum = new MecanumDrive(FLeft, BLeft, FRight, BRight);
  //other motors
  WPI_TalonSRX intakeLeft = new WPI_TalonSRX(4);
  WPI_TalonSRX intakeRight = new WPI_TalonSRX(8);
  WPI_TalonSRX intakeBelt = new WPI_TalonSRX(7);
  WPI_TalonSRX shooter = new WPI_TalonSRX(6);
  WPI_TalonSRX climber = new WPI_TalonSRX(5);
  //motions profiling 
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10, 20);
  TrapezoidProfile.State state = new TrapezoidProfile.State(5, 0);
  TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10), new TrapezoidProfile.State(5, 0), new TrapezoidProfile.State(0, 0));

  Joystick joy = new Joystick(0);
  Joystick bitterness = new Joystick(1);//needed to be different

  //Encoder Converter
  double distEncode = 26465/18; //26,465 units per 18 inches
  double sideEncode = 2065; 
  //Encoder Double Values
  double bLeftPos;
  double bRightPos;
  double fLeftPos;
  double fRightPos;
  double navXV;
  //joystick dead zone double values
  double joyX;
  double joyY;

  //sensors
  AHRS navX = new AHRS(Port.kMXP);

  // joy button mapping 
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
  // bitterness button mapping
  int b = 1;
  int l = 2;
  int r = 3;
  int t = 4;
  int RB = 10;
  int LB = 9;
  int LBlueT = 5;
  int LBlueB = 6;
  int RBlueT = 7;
  int RBlueB = 8;

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

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //Pneumatics
  Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  Solenoid wave = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  // Limit Switches
  AnalogInput upperLim = new AnalogInput(1);
  AnalogInput lowerLim = new AnalogInput(0);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {

    CameraServer.startAutomaticCapture();
    
    m_robotContainer = new RobotContainer();

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
    
    // Compressor
    compressor.enableDigital();

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
    //falcon 500 encoders' values set-up
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
    //Dashboard options for autonomous
    SmartDashboard.putData("Auto choices", m_chooser);
    m_chooser.addOption("Test", Test);
    m_chooser.addOption("OneMove", OneMove);
    m_chooser.addOption("None", None);
    m_chooser.addOption("twoBalls", twoBalls);
    m_chooser.addOption("threeBalls", threeBalls);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    timer.start();

    BLeft.setSelectedSensorPosition(0);
    BRight.setSelectedSensorPosition(0);
    FLeft.setSelectedSensorPosition(0);
    FRight.setSelectedSensorPosition(0);

    x = 1;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_autoSelected = m_chooser.getSelected();
    switch(m_autoSelected){
      //test autonomous - goes backwards 24 inches
      case Test:
        if(FLeft.getSelectedSensorPosition() > -24*distEncode){
          mecanum.driveCartesian(-0.25, 0, 0);
        }else{
          mecanum.driveCartesian(0, 0, 0);
        }
      break;
      case OneMove:
        if(x == 1 && timer.get() < 2){
          shooter.set(1);
        }else if(x == 1){
          shooter.set(0);
          x = 2;
        }
        if(BRight.getSelectedSensorPosition() < 80*distEncode && x == 2){
          mecanum.driveCartesian(0.33, 0, 0);
        }else if(x == 2){
          mecanum.driveCartesian(0, 0, 0);
          x = 3;
        }
      break;
      //oneMove
      // case OneMove:
      //   if(BRight.getSelectedSensorPosition() > -24*distEncode && x == 1){
      //     mecanum.driveCartesian(0.25, 0, 0);
      //   }else if(x == 1){
      //     mecanum.driveCartesian(0, 0, 0);
      //     BRight.setSelectedSensorPosition(0);
      //     x = 2;
      //   }
      //   if(BRight.getSelectedSensorPosition() < 15*sideEncode && x == 2){
      //     mecanum.driveCartesian(0, 0.25, 0);
      //   }else if(x == 2){
      //     mecanum.driveCartesian(0, 0, 0);
      //     x = 3;
      //   }
      //break;

      //threeBalls autonomous
      case threeBalls:
        if(x == 1 && timer.get() < 3){
          mecanum.driveCartesian(0, 0, 0);
          Shooter();
        }else if(x == 1){
          x = 2;
        }
        if(BRight.getSelectedSensorPosition() > -125*distEncode && x == 2){
          mecanum.driveCartesian(0.5, 0, 0);
        }else if(x == 2){
          mecanum.driveCartesian(0, 0, 0);
          BRight.setSelectedSensorPosition(0);
          x = 3;
          timer.reset();
        }
        if(x == 3 && timer.get() < 3){
          mecanum.driveCartesian(0, 0, 0);
          Intake();
        }else if(x == 3){
          x = 4;
        }
        if(BRight.getSelectedSensorPosition() < 125*distEncode && x == 4){
          mecanum.driveCartesian(0.5, 0, 0);
          timer.reset();
        }else if(x == 4 && timer.get() < 3){
          mecanum.driveCartesian(0, 0, 0);
          Shooter();
          x = 5;
        }
        if(navX.getAngle() < 65 && x == 5){
          mecanum.driveCartesian(0, 0.5, 0);
        }else if(x == 5){
          mecanum.driveCartesian(0, 0, 0);
          x = 6;
          BRight.setSelectedSensorPosition(0);
        }
        if(BRight.getSelectedSensorPosition() < -140*distEncode && x == 6){
          mecanum.driveCartesian(0.5, 0, 0);
        }else if(x == 6){
          mecanum.driveCartesian(0, 0, 0);
          x = 7;
          timer.reset();
        }
        if(x == 7 && timer.get() < 3){
          mecanum.driveCartesian(0, 0, 0);
          Intake();
        }else if(x == 7){
          BRight.setSelectedSensorPosition(0);
          timer.reset();
          x = 8;
        }
        if(BRight.getSelectedSensorPosition() < 140*distEncode && x == 8){
          mecanum.driveCartesian(-0.5, 0, 0);
        }else if(x == 8 && timer.get() < 3){
          mecanum.driveCartesian(0, 0, 0);
          Shooter();
          x = 9;
        }

      break;
      //twoBalls autonomous
      case twoBalls:
        if(x == 1 && timer.get() < 2){
          shooter.set(1);
          intakeBelt.set(1);
          mecanum.driveCartesian(0, 0, 0);
        }else if(x == 1){
          shooter.set(0);
          intakeBelt.set(0);
          x = 2;
        }
        if(BRight.getSelectedSensorPosition() < 95*distEncode && x == 2){
          mecanum.driveCartesian(0.33, 0, 0);
        }else if(x == 2){
          mecanum.driveCartesian(0, 0, 0);
          BRight.setSelectedSensorPosition(0);
          x = 3;
          timer.reset();
        }
        if(x == 3 && timer.get() < 2){
          intakeLeft.set(-1);
          intakeRight.set(1);
          intakeBelt.set(1);
          mecanum.driveCartesian(0, 0, 0);
        }else if(x == 3){
          intakeLeft.set(0);
          intakeRight.set(0);
          intakeBelt.set(0);
          BRight.setSelectedSensorPosition(0);
          x = 4;
        }
        if(BRight.getSelectedSensorPosition() > -103*distEncode && x == 4){
          mecanum.driveCartesian(-0.33, 0, 0);
        }else if(x == 4){
          mecanum.driveCartesian(0, 0, 0);
          timer.reset();
          x = 5;
        }
        if(x == 5 && timer.get() < 1){
          mecanum.driveCartesian(0, 0, 0);
        }else if (x == 5){
          x = 6;
        }
        if(x == 6 && timer.get() < 2){
          mecanum.driveCartesian(0, 0, 0);
          shooter.set(1);
          intakeBelt.set(1);
        }else if(x == 6){
          shooter.set(0);
          intakeBelt.set(0);
          x = 7;
        }
        if(x == 7 && BRight.getSelectedSensorPosition() < 90){
          mecanum.driveCartesian(0.33, 0, 0);
        }else if(x == 7){
          mecanum.driveCartesian(0, 0, 0);
          x = 8;
        }
      break;
      default:
        mecanum.driveCartesian(0, 0, 0);
        System.out.println("default :]");
      break;
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

    //resetting encoders and navX
    if(joy.getRawButton(Fire)){
      BLeft.setSelectedSensorPosition(0);
      BRight.setSelectedSensorPosition(0);
      FLeft.setSelectedSensorPosition(0);
      FRight.setSelectedSensorPosition(0);

      navX.reset();
    }
    // //set-up of dead zone for joystick
    // if(Math.abs(joy.getRawAxis(axisX)) > 0.05){
    //   joyX = joy.getRawAxis(axisX);
    // }else{
    //   joyX = 0;
    // }
    // if(Math.abs(joy.getRawAxis(axisY)) > 0.05){
    //   joyY = joy.getRawAxis(axisY);
    // }else{
    //   joyY = 0;
    // }
    mecanum.driveCartesian(-1*joy.getRawAxis(axisY), joy.getRawAxis(axisX), -1*joy.getRawAxis(rotZ));
    Intake();
    Shooter();
    Climber();
  }
  public void Intake(){
    if(joy.getRawButton(trigger)){
      intakeLeft.set(-1);
      intakeRight.set(1);
      intakeBelt.set(1);
    }else if(joy.getRawButton(pink)){
      intakeLeft.set(1);//combine so the two elses don't clash
      intakeRight.set(-1);
      intakeBelt.set(-1);
    }else{
      intakeLeft.set(0);
      intakeRight.set(0);
      intakeBelt.set(0);
    }
  }

  public void Shooter(){
    if(joy.getRawButton(Fire)){
      shooter.set(1);
      intakeBelt.set(1);
    }else{
      shooter.set(0);
    }
  }

  int flagPos = 0;
  public void Climber(){
    if((joy.getRawButton(E) || bitterness.getRawButton(r)) && upperLim.getVoltage() < 4 ){
      climber.set(1);//up
    }else if((joy.getRawButton(i) || bitterness.getRawButton(b)) && lowerLim.getVoltage() < 4){
      climber.set(-1);//down
    }else{
      climber.set(0);
    }

    SmartDashboard.putNumber("Upper Limit Voltage", upperLim.getVoltage());

    //toggle button function for moving the arm 
    // if(joy.getRawButton(A) && flagPos == 0){
    //   wave.set(true);
    // }else if(!joy.getRawButton(A) && flagPos == 0){
    //   flagPos = 1;
    // }else if(joy.getRawButton(A) && flagPos == 1){
    //   wave.set(false);
    // }else if(!joy.getRawButton(A) && flagPos == 1){
    //   flagPos = 0;
    // }

    if(joy.getRawButton(A) || bitterness.getRawButton(t)){
      wave.set(true);
    }else if(joy.getRawButton(B) || bitterness.getRawButton(l)){
      wave.set(false);
    }
  }

  public void AutoClimb(){

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

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private boolean camButton = false;
  private Command m_autonomousCommand;
  private  RobotContainer m_robotContainer;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private final AHRS ahrs = new AHRS(SPI.Port.kMXP); 
  private UsbCamera cam0;
  private UsbCamera cam1;
  private VideoSink server;
  public static final Joystick m_joystick = AutoConstants.m_driverJoystick;
  public static final XboxController m_xboxController = AutoConstants.m_driverController;

  private boolean leftBumpPress = false;

 // public static final AHRS ahrs = new AHRS(SPI.Port.kMXP); 
 // private static final PowerDistributionPanel PDP = new PowerDistributionPanel(0);
 // public final Encoder m_leftDriveEncoder = new Encoder(0,1);
  //public final Encoder m_rightDriveEncoder = new Encoder(2,3);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { 
    
    
      //This defines the camera.
      //An issue was that they both had to be plugged into the first USB port on the roborio through a USB hub
      //We also set the FPS and the resolution of each of the cameras.
      //To troubleshoot, one can SSH into the RoboRio, go to /dev and do "ls | grep "video""
      cam0 = CameraServer.getInstance().startAutomaticCapture(0);
      cam0.setFPS(15);
      cam0.setResolution(320,240);
      cam1 = CameraServer.getInstance().startAutomaticCapture(1);
      cam1.setFPS(15);
      cam1.setResolution(320,240);
      server = CameraServer.getInstance().getServer();

      
    
    m_robotContainer = new RobotContainer();
    
    /*
    m_leftDriveEncoder.setSamplesToAverage(5);
    m_rightDriveEncoder.setSamplesToAverage(5);
    //EVERYTHING CHANGES EXCEPT FOR THE 1.5. 1.5 REPRESENTS THE INCHES IN THE RADIUS OF THE WEEL. 
    m_leftDriveEncoder.setDistancePerPulse(1.0 /360.0 *2.0 * Math.PI * 6);
    m_rightDriveEncoder.setDistancePerPulse(1.0 /360.0 *2.0 * Math.PI * 6);
      //Set 1 inch per second to mean stopped. NEEDS TO BE TESTED/CHANGED
      m_leftDriveEncoder.setMinRate(0);
      m_rightDriveEncoder.setMinRate(0);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    ahrs.zeroYaw();
    m_robotContainer = new RobotContainer();
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);*/
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    //This will toggle the camera streams depending on whether or not a button on the joystick is pressed.
    //if one wanted three cameras, then the boolean camButton can be replaced with an int that wraps around
    //Button two is the button the thumb falls on when pressing t
    if(m_joystick.getRawButton(2)){
      camButton = !camButton;
    }
    if(m_joystick.getRawButton(7))
        camButton=true;
    if(m_joystick.getRawButton(8))
      camButton=false;
    //This will change the source beteen the two cameras
    if(camButton){
      server.setSource(cam0);
    }else{
      server.setSource(cam1);
    }

    if(m_xboxController.getBumperPressed(Hand.kLeft) == true){
      leftBumpPress = !leftBumpPress;
    }
    if(leftBumpPress){
      SmartDashboard.putString("Bottom Elevator", "Forward");
    } else {
      SmartDashboard.putString("Bottom Elevator", "Reverse");
    }

    //CameraServer.getInstance().addServer("Server");
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    //SmartDashboard.putNumber("Joystick X value", joystick1.getX());
    //SmartDashboard.putNumber("Joystick Y value", joystick.getY());
   /* SmartDashboard.putNumber("Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("Current", PDP.getTotalCurrent());
    SmartDashboard.putNumber("Temperature", PDP.getTemperature());
    SmartDashboard.putNumber("Right Drive", m_rightDriveEncoder.getDistance());
    SmartDashboard.putNumber("Left Drive", m_leftDriveEncoder.getDistance());*/
    CommandScheduler.getInstance().run();
    /*
    Color detectedColor = m_colorSensor.getColor();
    String colorString = "";
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget){
      colorString = "blue";
    }else if (match.color == kGreenTarget){
      colorString = "green";
    }else if (match.color == kRedTarget){
      colorString = "red";
    }else if (match.color == kYellowTarget){
      colorString = "yellow";
    }
    */
    //System.out.println("Detected Color" + colorString);
    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() { 
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
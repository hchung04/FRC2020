/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private double m_LimelightDriveCommand = 0;
  private double m_LimelightSteerCommand = 0;
  private boolean m_LimelightHasValidTarget;


  private boolean bPressed;
  private boolean triggerPressed = false;
  private boolean throttleMode;

  //gyros
  private int targetAngle = 0;
  private int angle = 0;
  private int error = 0;
  private int correctedError = 0;
  double P, I, D = 0;
  double Ad = 0;
  int prevError = 0;

  private SpeedController m_rearLeft = new PWMVictorSPX(AutoConstants.rearLeftDrive);
  private SpeedController m_frontLeft = new PWMVictorSPX(AutoConstants.frontLeftDrive);
  private SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

  private SpeedController m_rearRight = new PWMVictorSPX(AutoConstants.rearRightDrive);
  private SpeedController m_frontRight = new PWMVictorSPX(AutoConstants.frontRightDrive);
  private SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  
//ENCODERS:
private Encoder m_leftEncoder =
      new Encoder(AutoConstants.kLeftEncoderPorts[0], AutoConstants.kLeftEncoderPorts[1],
                  AutoConstants.kLeftEncoderReversed);

private Encoder m_rightEncoder =
 new Encoder(AutoConstants.kRightEncoderPorts[0], AutoConstants.kRightEncoderPorts[1],
  AutoConstants.kRightEncoderReversed);
  

// The gyro sensor
public static final AHRS ahrs = new AHRS(SPI.Port.kMXP); 

// Odometry class for tracking robot pose
private final DifferentialDriveOdometry m_odometry;


  
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left.setVoltage(leftVolts);
    m_right.setVoltage(-rightVolts);
    m_drive.feed();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public double getDistance() {
    double left =  m_leftEncoder.getDistance();
    double right = m_rightEncoder.getDistance();
    if(left==0&&right==0)
      return 0;
    if(left==0)
        return right;
    if(right==0)
        return left;
    return (left+right)/2;
  }
  public double getHeading() {
    return ahrs.getYaw();
  } 
  public DriveSubsystem() {
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(ahrs.getYaw()));
    m_leftEncoder.setSamplesToAverage(3);
    m_rightEncoder.setSamplesToAverage(3);
    m_leftEncoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 3);
    m_rightEncoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 3);
    m_leftEncoder.setMinRate(0);
    m_rightEncoder.setMinRate(0);
    ahrs.reset();
  }
  public void gyroReset(){
    ahrs.reset();
  }
      public void arcadeDriveSimple(double y, double x){
        arcadeDrive(y , x , false);
      }
      public void arcadeDrive(double y, double x) {
        /*
        SmartDashboard.putNumber("RIGHT",m_rightEncoder.getDistance());
        SmartDashboard.putNumber("LEFT",m_leftEncoder.getDistance());
        angle = (int) ahrs.getYaw();
        System.out.println(angle);
        if(angle < 0)
          angle = angle + 360;

        //Getting the target angle from arcTan
        targetAngle = (int)((180 / Math.PI) * Math.atan2(y, x));
        if(targetAngle < 0)
          targetAngle = 360 + targetAngle;  
  


        //Calculated initial error
        error = targetAngle - angle;  
        
        //Correcting the error to the expected value
        correctedError = error;
        if(error < -180)
          correctedError = 360 + error;
        else if (error > 180)
          correctedError = error - 360;
        
        // I & D calculations
        I = AutoConstants.kI * (I + correctedError);
  
        D = AutoConstants.kD * (correctedError - prevError);
  
        // Final adjustment summation with P calculation
        Ad = AutoConstants.kP * (double)correctedError + I + D;
  
        //Sets prev error for next run
        prevError = correctedError;
        SmartDashboard.putNumber("YAW", (int) ahrs.getYaw());
        //Driving forward
        if(angle < 180){
        //  System.out.println("X:   " + x);
        //  System.out.println("x+ad:" + x + Ad);
        
        arcadeDrive(y, x + Ad, false);
         } else {
         // System.out.println("X:  " + x);
         // System.out.println("x+ad  " + x + Ad);
          arcadeDrive(y, x - Ad, false);
         }
        */
        
        
        if(RobotContainer.getXboxController().getBButtonPressed()){
          bPressed = true;
        }
        if(RobotContainer.getXboxController().getBButtonReleased()){
          bPressed = false;
        }
        if(RobotContainer.getJoystick().getTriggerPressed()){
          triggerPressed = !triggerPressed;
        }
   

       
        if(RobotContainer.getJoystick().getThrottle() < 0){
          throttleMode = true;
        }
        if(RobotContainer.getJoystick().getThrottle() > 0){
          throttleMode = false;
        }

        System.out.println(bPressed);

       

        

        

        if(bPressed){
          if(triggerPressed){
            updateLimelightTrackingLong(y, x);
            arcadeDrive(-1 * m_LimelightDriveCommand, -1 * m_LimelightSteerCommand, false);
          } else {
            updateLimelightTrackingShort(y, x);
            arcadeDrive(-1 * m_LimelightDriveCommand, -1 * m_LimelightSteerCommand, false);
          }
          
        } else {
          m_LimelightDriveCommand = 0;
          m_LimelightSteerCommand = 0;
          if(throttleMode){
            arcadeDrive(y * 0.6, x * 0.6, false);
          } else {
            arcadeDrive(y * -0.6, x * 0.6, false);
          }
         }

          
          
         /*
           if(-1 * RobotContainer.getJoystick().getRawAxis(2) > 0){
              arcadeDrive(y * 0.6, -x * 0.6, false);
            } else {
              arcadeDrive(-y * 0.6, x * 0.6, false);
            }
          */

         /*
         if(RobotContainer.getXboxController().getStickButtonPressed(Hand.kLeft)){
          leftStickPress = true;
         }
         if(RobotContainer.getXboxController().getStickButtonReleased(Hand.kLeft)){
          leftStickPress = false;
         }
         if(leftStickPress){
          arcadeDrive(y, x, false);
         } else {
           arcadeDrive(0.5* y, 0.5 * x, false);
         }*/
      }
     
     
    

    public void arcadeDrive(double y, double x, boolean quickTurn) {
      if(RobotContainer.getXboxController().getTriggerAxis(Hand.kLeft) > .99){
        System.out.println("PRESSED LEFT");
      }
        m_drive.arcadeDrive(y, x, quickTurn);
    }
    public void orientToShoot(){
      int expectedAngle = 0;//need
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(1);
      System.out.println(tx);
      while (tx != expectedAngle){
          if (tx >= 0){
              m_left.set(.5);
          }else if(tx < 0){
              m_right.set(.5);
          }
      }
      double expectedArea = 2.6;//need
      double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(1);
      System.out.println(ta);
      while (ta != expectedArea){
          if (ta > expectedArea){
              m_drive.arcadeDrive(-0.5, 0);
          }else if(ta < expectedArea){
            m_drive.arcadeDrive(0.5, 0);
          }
      }
    }
    public void updateLimelightTrackingShort(double y, double x){
      m_LimelightDriveCommand = y;
      m_LimelightSteerCommand = x;
      final double STEER_K = 0.07;
      final double DRIVE_K = 1.00;
      final double DESIRED_TARGET_ANGLE = 1.06;
      final double DESIRED_TARGET_AREA = 2.561;

      double tv =NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(1);
      double tx =NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(1);
      double ty =NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(1);
      double ta =NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(1);

      if(tv < 1.0){
        m_LimelightHasValidTarget = false;
        m_LimelightDriveCommand = y;
        m_LimelightSteerCommand = x;
        return;
      } else {
        m_LimelightHasValidTarget = true;

        m_LimelightSteerCommand = (DESIRED_TARGET_ANGLE - tx) * STEER_K;

        m_LimelightDriveCommand = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        
      }
    }

    public void updateLimelightTrackingLong(double y, double x){
      m_LimelightDriveCommand = y;
      m_LimelightSteerCommand = x;
      final double STEER_K = 0.07;
      final double DRIVE_K = 1.00;
      final double DESIRED_TARGET_ANGLE = 1.06;
      final double DESIRED_TARGET_AREA = 2.561;

      double tv =NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(1);
      double tx =NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(1);
      double ty =NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(1);
      double ta =NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(1);

      if(tv < 1.0){
        m_LimelightHasValidTarget = false;
        m_LimelightDriveCommand = y;
        m_LimelightSteerCommand = x;
        return;
      } else {
        m_LimelightHasValidTarget = true;

        m_LimelightSteerCommand = (DESIRED_TARGET_ANGLE - tx) * STEER_K;

        m_LimelightDriveCommand = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        
      }
    }

    public void fullStop(){
      m_right.set(0);
      m_left.set(0);
      m_drive.arcadeDrive(0, 0);
    }

    public void stop() {
      m_drive.arcadeDrive(0, 0);
    }
  @Override
  public void periodic() {
      // Update the odometry in the periodic block
  /*    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
                        m_rightEncoder.getDistance());*/
  }
}
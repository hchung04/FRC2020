/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
public final class Constants {

    public static final class AutoConstants{
        public static final XboxController m_driverController = new XboxController(1);
        public static final Joystick m_driverJoystick = new Joystick(2);
        
        
        public static final double kP = 1/3600;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kDistance = 100;
        public static final double kSpeed = 0.7;
        public static final int controlPanelport = 8;
        public static final int IntakePort = 7;
        public static final int OuttakePort = 4 ;
        public static final int elevatorBottom = 5;
        public static final int elevatorTop = 6;
        public static final int rearLeftDrive = 0;
        public static final int frontLeftDrive = 1;
        public static final int rearRightDrive = 2;
        public static final int frontRightDrive = 3;
        /*
        private SpeedController m_rearLeft = new PWMVictorSPX(0);
        private SpeedController m_frontLeft = new PWMVictorSPX(1);
        private SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
      
        private SpeedController m_rearRight = new PWMVictorSPX(2);
        private SpeedController m_frontRight = new PWMVictorSPX(3);
        private SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
      
        private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);*/


        //BEN DRIVE CONSTANTS
        public static final double ksVolts=5;
        public static final double kvVoltSecondsPerMeter=5;
        public static final double kaVoltSecondsSquaredPerMeter=5;
    

        public static final double kPDriveVel=5;

        public static final double kTrackwidthMeters = 0.55;
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond=5 ;
        public static final double kMaxAccelerationMetersPerSecondSquared=5;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final int[] kLeftEncoderPorts= {0,1};
        public static final int[] kRightEncoderPorts= {2,3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;

        public static final double kEncoderDistancePerPulse=5;
    }

}
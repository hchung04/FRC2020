/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private SpeedController m_frontLeft = new PWMVictorSPX(1);
  private SpeedController m_rearLeft = new PWMVictorSPX(2);
  private SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);

  private SpeedController m_frontRight = new PWMVictorSPX(3);
  private SpeedController m_rearRight = new PWMVictorSPX(4);
  private SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);

  private DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  
  public DriveSubsystem() {

    /*private final SpeedController bottomLeft = new PWMVictorSPX(0);
    private final SpeedController topLeft = Constants.driveSubsystemTopLeft;
    private final SpeedControllerGroup left = Constants.driveSubsystemLeft;
    private final SpeedController bottomRight = Constants.driveSubsystemBottomRight;
    private final SpeedController topRight = Constants.driveSubsystemTopRight;
    private final SpeedControllerGroup right = Constants.driveSubsystemRight;
    private final DifferentialDrive differentialDrive1 = Constants.driveSubsystemDifferentialDrive1;*/

  }

  public void takeJoystickInput(Joystick joystick) {
    double sensitivity=((joystick.getThrottle()*-1)/8)+0.875;
    m_drive.arcadeDrive(joystick.getY()*sensitivity, joystick.getX()*sensitivity);
}

  public void stop() {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallIntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private SpeedController m_succController = new PWMVictorSPX(5);
  
  public BallIntakeSubsystem() {

  }

  public void ballTake() {
    System.out.println("fdws");
      m_succController.set(1);
  }

  public void stop() {
    m_succController.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
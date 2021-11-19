/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorUpSubsystem extends SubsystemBase {
  //Elevator Up Subsystem
  private PWMSparkMax m_top = new PWMSparkMax(AutoConstants.elevatorTop);
  private boolean bumpPress = false;

  public ElevatorUpSubsystem() {
  }

  public void goUp() {

    if(RobotContainer.getXboxController().getBumperPressed(Hand.kRight)){
      bumpPress = !bumpPress;
    }
    //The bump press functions to reverse the motor.
    if(bumpPress){
      SmartDashboard.putString("Top Elevator", "Reverse");
      m_top.set(0.3);
    } else {
      SmartDashboard.putString("Top Elevator", "Forward");
      m_top.set(-0.3);
    }
  }
  public void stop() {
    m_top.set(0);
  }

  @Override
  public void periodic() {
  }
}
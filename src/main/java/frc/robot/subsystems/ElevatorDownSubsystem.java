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

public class ElevatorDownSubsystem extends SubsystemBase {
  /**
   *Creates the Elevator Down Subsystem
   */

  private PWMSparkMax m_bottom = new PWMSparkMax(AutoConstants.elevatorBottom);
  private boolean bumpPress = false;

  public ElevatorDownSubsystem() {
  }

  public void goDown(){

    if(RobotContainer.getXboxController().getBumperPressed(Hand.kLeft)){
      bumpPress = !bumpPress;
    }

    if(bumpPress){
      SmartDashboard.putString("Bottom Elevator", "Forward");
      m_bottom.set(-0.3);
    } else {
      SmartDashboard.putString("Bottom Elevator", "Reverse");
      m_bottom.set(0.3);
    }
  }
  public void stop() {
    m_bottom.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
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
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlPanelSubsystem extends SubsystemBase {
  /**
   * Creates control panel subsystem
   */
  private final SpeedController panelController = new PWMVictorSPX(6);

  //INSERT THE TWO PORTS HERE that the encoder uses
  private final Encoder m_Encoder = new Encoder(2,3,false);
  
  
 
  //
  private double inches;
  
  

  public ControlPanelSubsystem() {
    //CAN CHANGE THIS
  m_Encoder.setSamplesToAverage(5);
  inches =0;
  //EVERYTHING CHANGES EXCEPT FOR THE 1.5. 1.5 REPRESENTS THE INCHES IN THE RADIUS OF THE WEEL. 
  m_Encoder.setDistancePerPulse(1.0 /360.0 *2.0 * Math.PI * 1.5);
    //Set 1 inch per second to mean stopped. NEEDS TO BE TESTED/CHANGED
    m_Encoder.setMinRate(0);  
 
  }
  public void rotate(double rev){
        panelController.set(.5);
        inches=rev;
  }
   public double findDistance(){
       return m_Encoder.getDistance();
   } 

  public void stop() {
   panelController.set(0);
   m_Encoder.reset();
  }
public boolean hasDoneRevolutions(){
    if(m_Encoder.getDistance() >= inches){
        return true;
    }
    return false;
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
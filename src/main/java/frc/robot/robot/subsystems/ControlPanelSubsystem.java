/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;


public class ControlPanelSubsystem extends SubsystemBase {
  /**
   * Creates control panel subsystem
   */
  private final SpeedController panelController = new PWMVictorSPX(6);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  //INSERT THE TWO PORTS HERE that the encoder uses
  private final Encoder m_Encoder = new Encoder(2,3,true);
   
  //
  private double inches;
  
  public ControlPanelSubsystem() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    //CAN CHANGE THIS
  m_Encoder.setSamplesToAverage(5);
  inches =0;
  //EVERYTHING CHANGES EXCEPT FOR THE 1.5. 1.5 REPRESENTS THE INCHES IN THE RADIUS OF THE WEEL. 
  m_Encoder.setDistancePerPulse(1.0 /360.0 *2.0 * Math.PI * 1.5);
    //Set 1 inch per second to mean stopped. NEEDS TO BE TESTED/CHANGED
    m_Encoder.setMinRate(0);  
 
  }
  public void goUntilColor(String choiceColor){
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
    if (colorString == choiceColor){
      stop();
    }
    else{
      rotateFreely();
    }
  }
  public void rotate(double rev){
        panelController.set(.5);
        inches=rev;
  }
  public void rotateFreely(){
    panelController.set(.2);
  }
   public double findDistance(){
       return m_Encoder.getDistance();
   } 

  public void stop() {
   panelController.set(0);
   m_Encoder.reset();
  }
public boolean hasDoneRevolutions(){
    if(Math.abs(m_Encoder.getDistance()) >= inches){
        return true;
    }
    return false;
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.BallIntake;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.smartdashboard.SmartDashboard;

public class BallIntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  // private PowerDistributionPanel pdp = new PowerDistributionPanel();
  private PWMTalonSRX m_Intake = new PWMTalonSRX(AutoConstants.IntakePort);
  public static final XboxController m_driverController = AutoConstants.m_driverController;
  private double lastSpeed = 0.0;
  private boolean stickPress = false;
  private final BallIntake intakeCommand = new BallIntake(this);


  public BallIntakeSubsystem() {
  }

  public void ballTake() {

    if(RobotContainer.getXboxController().getStartButtonPressed()){
      stickPress = !stickPress;
    }

    if(stickPress){
      SmartDashboard.putString("Intake", "In");
      System.out.println("Reverse");
      m_Intake.set(-0.7);
    } else {
      SmartDashboard.putString("Intake", "In");
      System.out.println("Forward");
      m_Intake.set(0.7);
    }

    /*
    if(RobotContainer.getXboxController().getStickButtonPressed(Hand.kRight)){
      m_Intake.set(-1);
    } else {
      m_Intake.set(1);
    }
    if(RobotContainer.getXboxController().getStickButtonReleased(Hand.kRight)){
      m_Intake.set(1);
    }*/
    // m_Intake.set(1);
  }

  public void stop() {
    m_Intake.set(0);
  }

  @Override
  public void periodic() {
    //final double leftTriggerStatus = m_driverController.getTriggerAxis(Hand.kLeft);
    final double leftTriggerStatus = RobotContainer.getXboxController().getTriggerAxis(Hand.kLeft);
    System.out.println(leftTriggerStatus);
    if (Math.abs(leftTriggerStatus)>=0.75){
      CommandScheduler.getInstance().schedule(intakeCommand);
    } else if (Math.abs(leftTriggerStatus)<=0.75){
      CommandScheduler.getInstance().cancel(intakeCommand);
    }
    // This method will be called once per scheduler run
  }
}

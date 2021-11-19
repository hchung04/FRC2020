/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ShootBallCommand;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class BallOuttakeSubsystem extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  // private PowerDistributionPanel pdp = new PowerDistributionPanel();
  private PWMSparkMax m_Shooter = new PWMSparkMax(AutoConstants.OuttakePort);
  public static final XboxController m_driverController = AutoConstants.m_driverController;
  private final ShootBallCommand intakeCommand = new ShootBallCommand(this);
  
  public BallOuttakeSubsystem() {
  }

  public void ballShoot() {
    
    m_Shooter.set(.96);
  }

  public void stop() {
    m_Shooter.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final double rightTriggerStatus = m_driverController.getTriggerAxis(Hand.kRight);
    if (Math.abs(rightTriggerStatus) >= 0.75){
      CommandScheduler.getInstance().schedule(intakeCommand);
    } else if (Math.abs(rightTriggerStatus)<=0.75){
      CommandScheduler.getInstance().cancel(intakeCommand);
    }
  }
}
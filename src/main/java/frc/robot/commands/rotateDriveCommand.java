/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

//This command is supposed to rotate the robot to a set number of degress.
public class rotateDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
    private final double a;
  /**
   * @param subsystem The subsystem used by this command.
   */
  public rotateDriveCommand(DriveSubsystem subsystem,int angle) {
    a=angle;
    m_subsystem = subsystem;
    m_subsystem.gyroReset();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.arcadeDriveSimple(0,.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("ANGLE: " + m_subsystem.getHeading());
    if(a==180 && (m_subsystem.getHeading()>178 || m_subsystem.getHeading() < -178))
        return true; 
    return m_subsystem.getHeading() < (a+2) && m_subsystem.getHeading() > (a-2);
  }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.BallOuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
/**
 * An example command that uses an example subsystem.
 */
public class AutonomousShootBallCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final BallOuttakeSubsystem m_subsystem;
    private Timer t;
    private int seconds;
  /**
   * Creates a new subsystem
   *
   * @param subsystem The subsystem used by this command.
   */
  //This should turn the shooter on for s number of seconds. 
  public AutonomousShootBallCommand(BallOuttakeSubsystem subsystem, int s) {
    seconds =s;
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
 
  // Called when the command is initially scheduled.
  //Instantiate the timer and start it
  @Override
  public void initialize() {
      t = new Timer();
      t.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_subsystem.ballShoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_subsystem.stop();
  }

  // Returns true when the command should end.
  //This will check to see if the timer is done to see if we should end the command. 
  @Override
  public boolean isFinished() {
    return t.get()>seconds;
  }
}

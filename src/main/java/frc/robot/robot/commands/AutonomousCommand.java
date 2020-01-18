/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * An example command that uses an example subsystem.
 */
public class AutonomousCommand extends SequentialCommandGroup {


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousCommand(DriveSubsystem subsystemDrive, BallIntakeSubsystem subsystemIntake, ControlPanelSubsystem controlPanel){

    addCommands(
        new rotateCommand(controlPanel)
        //new AutonomousDrive(subsystemDrive, AutoConstants.kDistance, AutoConstants.kSpeed), new BallSucc(subsystemIntake)
    );

  }
}
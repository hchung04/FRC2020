/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.BallSucc;
 import frc.robot.commands.rotateCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallIntakeSubsystem;
 import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final DigitalInput limitSwitch= new DigitalInput(AutoConstants.switchPort);
  private final ArcadeDrive driveCommand = new ArcadeDrive(driveSubsystem, this::getLeftY, this::getLeftX);

  public static final XboxController m_driverController = new XboxController(0);

  private final BallIntakeSubsystem intakeSubsystem = new BallIntakeSubsystem();

  private final BallSucc intakeCommand = new BallSucc(intakeSubsystem);

 private final ControlPanelSubsystem panelSubsytem = new ControlPanelSubsystem();
 private final rotateCommand panelCommand = new rotateCommand(panelSubsytem);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(driveCommand);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, XboxController.Button.kA.value).whenHeld(intakeCommand);
   new JoystickButton(m_driverController, XboxController.Button.kX.value).whenPressed(panelCommand);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return panelCommand; //SequentialCommandGroup(panelCommand, new WaitCommand(5),panelCommand);  //SequentialCommandGroup(panelCommand, new WaitCommand(10), panelCommand);
  }

  public static XboxController getXboxController(){
    return m_driverController;
  }

  public double getLeftX() {
    return m_driverController.getX(Hand.kLeft);
}

  public double getLeftY() {
  return m_driverController.getY(Hand.kLeft);
}
  public void buttonPressed(){
      boolean resultSwitch = !(limitSwitch.get());
      System.out.println("Button Pressed: " + resultSwitch);
  }
}

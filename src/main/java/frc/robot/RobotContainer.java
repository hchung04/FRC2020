/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.BallSucc;
 import frc.robot.commands.rotateCommand;
 import frc.robot.commands.ShootBallCommand;
 import frc.robot.commands.MovingDownCommand;
 import frc.robot.commands.waitCommand;
 import frc.robot.commands.MovingUpCommand;
 import frc.robot.commands.rotateDriveCommand;
import frc.robot.commands.updateLimelightTrackingLongCommand;
import frc.robot.commands.driveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.BallOuttakeSubsystem;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
 import frc.robot.subsystems.ControlPanelSubsystem;
 import frc.robot.subsystems.PnumadicSystem;
 import frc.robot.subsystems.ElevatorUpSubsystem;
 import frc.robot.subsystems.ElevatorDownSubsystem;
 import frc.robot.commands.BallStopSucc;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
/** 
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.PIDController;*/
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static final XboxController m_driverController = AutoConstants.m_driverController;
  public static final Joystick m_joystick = AutoConstants.m_driverJoystick;


  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ArcadeDrive driveCommand = new ArcadeDrive(driveSubsystem, this::getLeftY, this::getLeftX);

  private final BallIntakeSubsystem intakeSubsystem = new BallIntakeSubsystem();
  private final BallSucc intakeCommand = new BallSucc(intakeSubsystem);

  private static BallOuttakeSubsystem outtakeSubsystem = new BallOuttakeSubsystem();
  private static ShootBallCommand shootingCommand = new ShootBallCommand(outtakeSubsystem);

 private final ControlPanelSubsystem panelSubsytem = new ControlPanelSubsystem();
 private final rotateCommand panelCommand = new rotateCommand(panelSubsytem);

 private final ElevatorUpSubsystem elevatorUpSubsystem = new ElevatorUpSubsystem();
 private final MovingUpCommand movingUpCommand = new MovingUpCommand(elevatorUpSubsystem);

 private final ElevatorDownSubsystem elevatorDownSubsystem = new ElevatorDownSubsystem();
 private final MovingDownCommand movingDownCommand = new MovingDownCommand(elevatorDownSubsystem);



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
    new JoystickButton(m_driverController, XboxController.Button.kY.value).whileHeld(movingDownCommand);
    new JoystickButton(m_driverController, XboxController.Button.kX.value).whileHeld(movingUpCommand);
    new JoystickButton(m_driverController, XboxController.Button.kA.value).whileHeld(panelCommand);
    new JoystickButton(m_driverController, XboxController.Button.kStart.value).whileHeld(panelCommand);
    

  
    //new JoystickButton(armController, controlPanel).whenPressed(panelCommand);
  
  }

  public static XboxController getXboxController(){
    return m_driverController;
  }

  public static Joystick getJoystick(){
    return m_joystick;
  }

  public double getLeftX() {
    return m_driverController.getX(Hand.kLeft);
}

  public double getLeftY() {
  return m_driverController.getY(Hand.kLeft);
}


  public void buttonPressed(){
  }

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  
    return new SequentialCommandGroup(new driveCommand(driveSubsystem, 7), new updateLimelightTrackingLongCommand(driveSubsystem));
    
    
    
    /*
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(AutoConstants.ksVolts,
                                       AutoConstants.kvVoltSecondsPerMeter,
                                       AutoConstants.kaVoltSecondsSquaredPerMeter),
            AutoConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutoConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        driveSubsystem::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(AutoConstants.ksVolts,
                                   AutoConstants.kvVoltSecondsPerMeter,
                                   AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics,
        driveSubsystem::getWheelSpeeds,
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        new PIDController(AutoConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        driveSubsystem::tankDriveVolts,
        driveSubsystem
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));*/
  }
}
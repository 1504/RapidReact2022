// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Pathing.PathChooser;
import frc.robot.commands.Auton.AutonPhaseGroup;
import frc.robot.commands.Auton.FollowTrajectory;
import frc.robot.commands.Drive.Cartesian;
import frc.robot.commands.Drive.ReverseDrive;
import frc.robot.commands.Misc.DriveBack;
import frc.robot.commands.Shooter.DriveBall;
import frc.robot.commands.Shooter.PIDPewPew;
import frc.robot.commands.Winch.ToggleSolomon;
import frc.robot.commands.Winch.WinchPull;
import frc.robot.commands.Winch.WinchPush;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.MecanumDrivetrain;
import frc.robot.subsystems.PIDShooter;
import frc.robot.subsystems.Winch;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  ///Subsystems
  private final Drivetrain m_drive = new Drivetrain();
  private final MecanumDrivetrain _drive = new MecanumDrivetrain();
  private final PIDShooter p_shoot = new PIDShooter();
  private final Winch m_winch = new Winch();

  //Commands

  //Controllers
  private final Joystick _joystickOne = new Joystick(IOConstants.LEFT_JOYSTICK); //Controller for translation
  private final Joystick _joystickTwo = new Joystick(IOConstants.RIGHT_JOYSTICK); //Controller for rotation
  private final XboxController m_Controller = new XboxController(IOConstants.MAIN_CONTROLLER);

  //UsbCamera _cam = CameraServer.startAutomaticCapture();
  ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private PathChooser pathChooser = new PathChooser();

  public RobotContainer() {

    /*
    _cam.setResolution(360, 240);
    m_tab.add("Cam", _cam)
      .withPosition(0, 0)
      .withSize(3, 3);
*/
    configureButtonBindings();
    m_drive.setDefaultCommand(new Cartesian(m_drive, () -> m_Controller.getLeftY(),() -> m_Controller.getRightX(), () -> -m_Controller.getLeftX()));
    //m_winch.setDefaultCommand(new WinchPull(m_winch, () -> m_Controller.getLeftY()));
    //m_shoot.setDefaultCommand(new PewPew(m_shoot, _shooter.getY(), _shooter.getY()));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(m_Controller, XboxController.Button.kX.value).whenHeld(new PewPew(m_shoot));
    //new JoystickButton(m_Controller, XboxController.Button.kX.value).whenHeld(new PIDPewPew(p_shoot, -1, -1));
    new JoystickButton(m_Controller, XboxController.Button.kX.value).whenHeld(new PIDPewPew(p_shoot, 2100, 3550));
    new JoystickButton(m_Controller, XboxController.Button.kRightBumper.value).whenHeld(new DriveBall(p_shoot));
    new JoystickButton(m_Controller, XboxController.Button.kB.value).whenHeld(new PIDPewPew(p_shoot, -1000, -1000));
    new JoystickButton(m_Controller, XboxController.Button.kA.value).whenHeld(new PIDPewPew(p_shoot, 5500, -400));
    //new JoystickButton(m_Controller, XboxController.Button.kA.value).whenHeld(new PIDPewPew(p_shoot, 2100, 3550));
    new JoystickButton(m_Controller, XboxController.Button.kY.value).whenHeld(new WinchPull(m_winch)); //Climb boi
    new JoystickButton(m_Controller, XboxController.Button.kLeftBumper.value).whenHeld(new WinchPush(m_winch)); //Reset boi
    //new JoystickButton(m_Controller, XboxController.Button.kB.value).whenHeld(new PIDPewPew(p_shoot, 1000, 3000));
    new JoystickButton(m_Controller, XboxController.Button.kBack.value).whenHeld(new DriveBack(m_drive));
    new JoystickButton(_joystickOne, 1).whenHeld(new ReverseDrive(m_drive));
    //new JoystickButton(m_Controller, XboxController.Button.kStart.value).whenPressed(new ToggleSolomon(m_winch));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() {
    //return new AutonPhaseGroup(m_drive, p_shoot); 
    return new FollowTrajectory(_drive, pathChooser.getTrajectory());
  }
}

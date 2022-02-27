// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.commands.Auton.AutonPhaseGroup;
import frc.robot.commands.Drive.Cartesian;
import frc.robot.commands.Shooter.DriveBall;
import frc.robot.commands.Shooter.PIDPewPew;
import frc.robot.commands.Shooter.PewPew;
import frc.robot.commands.Winch.WinchPull;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PIDShooter;
import frc.robot.subsystems.PIDShooterBad;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Winch;
import edu.wpi.first.wpilibj2.command.Command;
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
  //private final Shooter m_shoot = new Shooter();
  private final PIDShooter p_shoot = new PIDShooter();
  private final Winch m_winch = new Winch();

  //Commands

  //Controllers
  private final Joystick _joystickOne = new Joystick(IOConstants.LEFT_JOYSTICK); //Controller for translation
  private final Joystick _joystickTwo = new Joystick(IOConstants.RIGHT_JOYSTICK); //Controller for rotation
  //private final Joystick _shooter = new Joystick(3);
  private final XboxController _shootController = new XboxController(IOConstants.SHOOT_CONTROLLER);

  
  public RobotContainer() {

    configureButtonBindings();
    m_drive.setDefaultCommand(new Cartesian(m_drive, () -> _joystickOne.getY(), () -> _joystickOne.getX(), () -> _joystickTwo.getX()));
    m_winch.setDefaultCommand(new WinchPull(m_winch, () -> _shootController.getLeftY()));
    //m_shoot.setDefaultCommand(new PewPew(m_shoot, _shooter.getY(), _shooter.getY()));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(_shootController, XboxController.Button.kX.value).whenHeld(new PewPew(m_shoot));
    new JoystickButton(_shootController, XboxController.Button.kX.value).whenHeld(new PIDPewPew(p_shoot));
    new JoystickButton(_shootController, XboxController.Button.kY.value).whenHeld(new DriveBall(p_shoot));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new AutonPhaseGroup(m_drive, p_shoot); //temp line; soon to be changed
  }
}

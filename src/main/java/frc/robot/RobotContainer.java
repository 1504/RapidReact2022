// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.Drive.Cartesian;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  ///Subsystems
  private final Drivetrain m_drive = new Drivetrain();

  //Commands
  

  //Controllers
  private final Joystick _joystickOne = new Joystick(IOConstants.LEFT_JOYSTICK); //Controller for translation
  private final Joystick _joystickTwo = new Joystick(IOConstants.RIGHT_JOYSTICK); //Controller for rotation

  
  public RobotContainer() {

    configureButtonBindings();
    m_drive.setDefaultCommand(new Cartesian(m_drive, () -> _joystickOne.getY(), () -> _joystickOne.getX(), () -> _joystickTwo.getX()));


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new Cartesian(m_drive, () -> 1f, () -> 1f, () -> 1f); //temp line; soon to be changed
  }
}

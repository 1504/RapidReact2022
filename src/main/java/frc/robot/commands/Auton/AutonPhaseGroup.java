// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PIDShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonPhaseGroup extends SequentialCommandGroup {
  
  private final Drivetrain _drive;
  private final PIDShooter _shoot;

  public AutonPhaseGroup(Drivetrain _d, PIDShooter _s) {
    _drive = _d;
    _shoot = _s;
    // addCommands(new FooCommand(), new BarCommand());
    System.out.println("Poggers");
    addCommands(
      new DriveDistance(_drive, 108),
      new AutonShoot(_shoot, 4)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class PewPew extends CommandBase {
  
  private final Shooter _shoot;

  public PewPew(Shooter s) {
    _shoot = s;
    addRequirements(_shoot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shoot.shoot();
    //System.out.println(_shoot.test());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(_shoot.test());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shoot.stopMotors();
    //System.out.println(_shoot.test());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

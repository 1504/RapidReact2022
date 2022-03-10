// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDShooter;

public class PIDPewPew extends CommandBase {
  
  private final PIDShooter _shoot;
  private final double topRPM;
  private final double botRPM;

  public PIDPewPew(PIDShooter s, double top, double bot) {
    _shoot = s;
    topRPM = top;
    botRPM = bot;
    //addRequirements(s);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(botRPM < 0) {
      _shoot.setBot();
    } else {
      _shoot.setBot(botRPM);
    }
    if(topRPM < 0) {
      _shoot.setTop();
    } else {
      _shoot.setTop(topRPM);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shoot.setBot(0);
    _shoot.setTop(0);
    _shoot.stopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

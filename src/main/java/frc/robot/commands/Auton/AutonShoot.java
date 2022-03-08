// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDShooter;

public class AutonShoot extends CommandBase {

  private final PIDShooter _shoot;
  private final Timer _timer;
  private final double _time;

  public AutonShoot(PIDShooter _s, double _t) {
    _timer = new Timer();
    _shoot = _s;
    _time = _t;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer.reset();
    _timer.start();
    _shoot.setTop(1000);
    _shoot.setBot(5000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (_timer.get() > 2) {
      _shoot.driveBall();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shoot.stopBall();
    _shoot.setTop(0);
    _shoot.setBot(0);
    _shoot.stopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _timer.get() > _time;
  }
}

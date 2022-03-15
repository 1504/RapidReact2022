// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.subsystems.MecanumDrivetrain;

public class FollowTrajectory extends CommandBase {
  
  private final MecanumDrivetrain _drive;
  private final Trajectory _trajectory;
  //private final TrajectoryConfig _trajectory_config;

  public FollowTrajectory(MecanumDrivetrain _d, Trajectory _t) {
    _drive = _d;
    _trajectory = _t;

    MecanumControllerCommand controllerCommand;

    addRequirements(_d);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

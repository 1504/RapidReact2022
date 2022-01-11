// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Cartesian extends CommandBase {
  
  private final Drivetrain _drive;
  private final DoubleSupplier _ySpeed;
  private final DoubleSupplier _xSpeed;
  private final DoubleSupplier _zRot;

  public Cartesian(Drivetrain _d, DoubleSupplier _y, DoubleSupplier _x, DoubleSupplier _z) {
    _drive = _d;
    _ySpeed = _y;
    _xSpeed = _x;
    _zRot = _z;
    addRequirements(_d);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _drive.cartesianDrive(_ySpeed.getAsDouble(), _xSpeed.getAsDouble(), _zRot.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

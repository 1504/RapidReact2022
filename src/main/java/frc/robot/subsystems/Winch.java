// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.WinchConstants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase {
  
  //private final Compressor _compressor;

  private final CANSparkMax _left;
  private final CANSparkMax _right;
  private boolean _up = true;

  public Winch() {
    _left = new CANSparkMax(WinchConstants.LEFT_WINCH, MotorType.kBrushless);
    _right = new CANSparkMax(WinchConstants.RIGHT_WINCH, MotorType.kBrushless);

    //_compressor = new Compressor();
  }

  public boolean direction() {
    return _up;
  }

  public void changeDirection() {
    _up = !_up;
  }

  //Should later be set to go up a certain distance with PID controllers
  public void extend(double _s) {
    _left.set(_s);
    _right.set(-_s);
  }

  public void contract() {
    _left.set(-.1);
    _right.set(.1);
  }

  public void stopMotors() {
    _left.stopMotor();
    _right.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

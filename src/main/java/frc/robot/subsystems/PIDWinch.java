// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.WinchConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDWinch extends SubsystemBase {
  
  private final CANSparkMax _left;
  private final CANSparkMax _right;

  private final RelativeEncoder _left_encoder;
  private final RelativeEncoder _right_encoder;

  private final PIDController _left_pid;
  private final PIDController _right_pid;

  double _l_p;
  double _r_p;

  ShuffleboardTab w_tab = Shuffleboard.getTab("PID Winch");
  NetworkTableEntry setpoint_left;
  NetworkTableEntry setpoint_right;
  NetworkTableEntry position_left;
  NetworkTableEntry position_right;


  public PIDWinch() {
    _left = new CANSparkMax(WinchConstants.LEFT_WINCH, MotorType.kBrushless);
    _right = new CANSparkMax(WinchConstants.RIGHT_WINCH, MotorType.kBrushless);

    _left_encoder = _left.getEncoder();
    _right_encoder = _right.getEncoder();

    _left_pid = new PIDController(0, 0, 0);
    _right_pid = new PIDController(0, 0, 0);

    position_left = w_tab.add("Left Position", getLeftPosition())
      .withPosition(1, 0)
      .withSize(3, 3)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();
    position_right = w_tab.add("Right Position", getRightPosition())
      .withPosition(5, 0)
      .withSize(3, 3)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();
    setpoint_left = w_tab.add("Left Winch Setpoint", 0)
      .withPosition(1, 3)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
    setpoint_right = w_tab.add("Right Winch Setpoint", 0)
      .withPosition(5, 3)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
    w_tab.add("Left PID", _left_pid)
      .withPosition(0, 0)
      .withSize(1, 2);
    w_tab.add("Right PID", _right_pid)
      .withPosition(0, 0)
      .withSize(1, 2);
  }

  public double getLeftPosition() {
    return _left_encoder.getPosition();
  }

  public double getRightPosition() {
    return _right_encoder.getPosition();
  }

  public void setLeft() {
    _left_pid.setSetpoint(_l_p);
  }
  public void setRight() {
    _right_pid.setSetpoint(_r_p);
  }

  @Override
  public void periodic() {
    position_left.setNumber(getLeftPosition());
    position_right.setNumber(getRightPosition());

    _l_p = setpoint_left.getDouble(0);
    _r_p = setpoint_right.getDouble(0);

    double left_out = _left_pid.calculate(getLeftPosition());
    double right_out = _right_pid.calculate(getRightPosition());

    _left.setVoltage(Math.min(left_out, .2));
    _right.setVoltage(Math.min(right_out, .2));
    // This method will be called once per scheduler run
  }
}

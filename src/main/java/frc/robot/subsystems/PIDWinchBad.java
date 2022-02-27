// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.WinchConstants;


/*

idk if im going to delete this one yet
still havent figured out if im gonna use 1 pid controller or 2 for winch

*/


public class PIDWinchBad extends PIDSubsystem {
  
  private final CANSparkMax _left;
  private final CANSparkMax _right;

  private final RelativeEncoder _left_encoder;
  private final RelativeEncoder _right_encoder;

  //private final SimpleMotorFeedforward _feed_forward;

  public PIDWinchBad() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

    _left = new CANSparkMax(WinchConstants.LEFT_WINCH, MotorType.kBrushless);
    _right = new CANSparkMax(WinchConstants.RIGHT_WINCH, MotorType.kBrushless);

    _left_encoder = _left.getEncoder();
    _right_encoder = _right.getEncoder();
    
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}

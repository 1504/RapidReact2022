// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.WinchConstants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase {
  
  // private final Compressor _compressor;
  // private final Solenoid _solomon;

  private final CANSparkMax _left;
  private final CANSparkMax _right;

  private final RelativeEncoder _left_encoder;
  private final RelativeEncoder _right_encoder;

  private boolean _up = true;
  private boolean _toggle = false;

  ShuffleboardTab w_tab = Shuffleboard.getTab("Winch");
  NetworkTableEntry positionLeft;
  NetworkTableEntry positionRight;

  public Winch() {
    _left = new CANSparkMax(WinchConstants.LEFT_WINCH, MotorType.kBrushless);
    _right = new CANSparkMax(WinchConstants.RIGHT_WINCH, MotorType.kBrushless);

    _left_encoder = _left.getEncoder();
    _right_encoder = _right.getEncoder();

    positionLeft = w_tab.add("Left Position", getLeftRPM())
      .withPosition(0,0)
      .withSize(1,1)
      .getEntry();
    positionRight = w_tab.add("Right Position", getLeftRPM())
      .withPosition(1,0)
      .withSize(1,1)
      .getEntry();
    
    // _compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    // _solomon = new Solenoid(PneumaticsModuleType.CTREPCM, WinchConstants.SOLOMON_PORT);
  }

  public void toggleSolenoid() {
    _toggle = !_toggle;
    // _solomon.set(_toggle);
  }

  public boolean direction() {
    return _up;
  }

  public void changeDirection() {
    _up = !_up;
  }

  /**
   * Gets the rpm of the left winch
   * @return the rpm of the left winch
   */
  public double getLeftRPM() {
    return _left_encoder.getVelocity();
  }
  /**
   * Gets the rpm of the right winch
   * @return the rpm of the right winch
   */
  public double getRightRPM() {
    return _right_encoder.getVelocity();
  }

  /**
   * Gets the position of the left winch
   * @return the position of the left winch
   */
  public double getLeftDistance() {
    return _left_encoder.getPosition();
  }
  /**
   * Gets the position of the right winch
   * @return the position of the right winch
   */
  public double getRightDistance() {
    return _right_encoder.getPosition();
  }

  //Should later be set to go up a certain distance with PID controllers
  public void extend(double _s) {
    _left.set(_s);
    _right.set(-_s);
  }
  public void extend() {
    _left.set(-.1);
    _right.set(-.1);
  }

  public void contract() {
    _left.set(.1);
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

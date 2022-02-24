// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ShootConstants;
import frc.robot.Utils.Presets;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDShooter extends PIDSubsystem {
  
  //private Presets _preset;
  //private boolean _t;

  private final CANSparkMax _top_shooter;
  private final CANSparkMax _bottom_shooter;

  private final RelativeEncoder _top_encoder;
  private final RelativeEncoder _bottom_encoder;

  //carriage utility mechanism
  private final WPI_TalonSRX _elevator;

  private final SimpleMotorFeedforward _feed_forward;

  ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  NetworkTableEntry rpm;


  public PIDShooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

    _top_shooter = new CANSparkMax(ShootConstants.TOP_SHOOTER, MotorType.kBrushless);
    _bottom_shooter = new CANSparkMax(ShootConstants.BOT_SHOOTER, MotorType.kBrushless);
  
    _top_encoder = _top_shooter.getEncoder();
    _bottom_encoder = _bottom_shooter.getEncoder();

    _elevator = new WPI_TalonSRX(ShootConstants.ELEVATOR_PORT);

    _bottom_shooter.setInverted(true);
    
    _feed_forward = new SimpleMotorFeedforward(0, 0);

    rpm = m_tab.add("Shooter rpm", getRPM()).withPosition(7,0).withSize(1,1).getEntry();

    m_tab.add("PID Controller", getController()).withPosition(6,0).withSize(1,2);

  }
  
  public double getRPM() {
    return (_top_encoder.getVelocity() + _bottom_encoder.getVelocity()) / 2;
  }

  public void startShooter() {
    enable();
  }

  public void stopShooter() {
    disable();
  }

  public void driveBall() {
    _elevator.set(0.1);
  }

  public void stopBall() {
    _elevator.stopMotor();
  }

  public void rawShoot() {
    _top_shooter.set(.1);
    _bottom_shooter.set(-.1);
  }

  public void stopShoot() {
    _top_shooter.stopMotor();
    _bottom_shooter.stopMotor();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    _top_shooter.setVoltage(output + _feed_forward.calculate(setpoint));
    _bottom_shooter.setVoltage(output + _feed_forward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getRPM();
  }

  @Override
  public void periodic() {
    rpm.setNumber(getRPM());
    super.periodic();
  }
}

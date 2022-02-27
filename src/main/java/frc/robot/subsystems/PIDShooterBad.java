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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDShooterBad extends PIDSubsystem {
  
  //private Presets _preset;
  //private boolean _t = true;

  private final CANSparkMax _top_shooter;
  private final CANSparkMax _bottom_shooter;

  private final RelativeEncoder _top_encoder;
  private final RelativeEncoder _bottom_encoder;

  //carriage utility mechanism
  private final WPI_TalonSRX _elevator;

  //private final SimpleMotorFeedforward _feed_forward;

  ShuffleboardTab s_tab = Shuffleboard.getTab("Shooter");
  NetworkTableEntry rpmTop;
  NetworkTableEntry rpmBot;
  NetworkTableEntry setpointTop;
  NetworkTableEntry setpointBot;

  double[][] setpoints = {{0,0}};
  int index = 0;

  double _t_s;
  double _b_s;


  public PIDShooterBad() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

    _top_shooter = new CANSparkMax(ShootConstants.TOP_SHOOTER, MotorType.kBrushless);
    _bottom_shooter = new CANSparkMax(ShootConstants.BOT_SHOOTER, MotorType.kBrushless);
  
    _top_encoder = _top_shooter.getEncoder();
    _bottom_encoder = _bottom_shooter.getEncoder();

    _elevator = new WPI_TalonSRX(ShootConstants.ELEVATOR_PORT);

    _bottom_shooter.setInverted(true);
    
    //_feed_forward = new SimpleMotorFeedforward(0, 0);

    rpmTop = s_tab.add("Top Shooter rpm", getTopRPM())
      .withPosition(1,0)
      .withSize(3,3)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();
    rpmBot = s_tab.add("Bottom Shooter rpm", getBotRPM())
      .withPosition(4,0)
      .withSize(3,3)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();
    setpointTop = s_tab.add("Top Shooter setpoint", 0)
      .withPosition(1, 3)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
    setpointBot = s_tab.add("Bottom Shooter setpoint", 0)
      .withPosition(4, 3)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();

    s_tab.add("PID Controller", getController())
      .withPosition(0,0)
      .withSize(1,2);

    /*
    For the tuning process, set p i and d all to 0
    increase p until output starts to oscillate around the setpoint
    increase d as much as possible without introducing jittering in the system response
    */


  }
  
  public double getBotRPM() {
    return (_bottom_encoder.getVelocity());
  }

  public double getTopRPM() {
    return (_top_encoder.getVelocity());
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
    _top_shooter.set(.63);
    _bottom_shooter.set(.83);
  }

  public void stopShoot() {
    _top_shooter.stopMotor();
    _bottom_shooter.stopMotor();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    _top_shooter.setVoltage(output);
    _bottom_shooter.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getTopRPM();
  }

  @Override
  public void periodic() {
    rpmTop.setNumber(getTopRPM());
    rpmBot.setNumber(getBotRPM());

    _t_s = setpointTop.getDouble(0);
    _b_s = setpointBot.getDouble(0);


    super.periodic();
  }
}

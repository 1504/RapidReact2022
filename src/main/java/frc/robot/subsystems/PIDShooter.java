// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShootConstants;

public class PIDShooter extends SubsystemBase {
  

  private final CANSparkMax _top_shooter;
  private final CANSparkMax _bottom_shooter;

  private final RelativeEncoder _top_encoder;
  private final RelativeEncoder _bottom_encoder;

  private final PIDController _PID_top;
  private final PIDController _PID_bot;

  //carriage utility mechanism
  private final WPI_TalonSRX _elevator;

  ShuffleboardTab s_tab = Shuffleboard.getTab("Shooter");
  NetworkTableEntry rpmTop;
  NetworkTableEntry rpmTopText;
  NetworkTableEntry rpmBot;
  NetworkTableEntry rpmBotText;

  NetworkTableEntry setpointTop;
  NetworkTableEntry setpointBot;

  double[][] setpoints = {{0,0}};
  int index = 0;

  double _t_s;
  double _b_s;

  private static final SimpleMotorFeedforward _feedforward_bot = new SimpleMotorFeedforward(ShootConstants.BOT_KS, ShootConstants.BOT_KV);
  private static final SimpleMotorFeedforward _feedforward_top = new SimpleMotorFeedforward(ShootConstants.TOP_KS, ShootConstants.TOP_KV);

  public PIDShooter() {
    _top_shooter = new CANSparkMax(ShootConstants.TOP_SHOOTER, MotorType.kBrushless);
    _bottom_shooter = new CANSparkMax(ShootConstants.BOT_SHOOTER, MotorType.kBrushless);
  
    _top_encoder = _top_shooter.getEncoder();
    _bottom_encoder = _bottom_shooter.getEncoder();

    _elevator = new WPI_TalonSRX(ShootConstants.ELEVATOR_PORT);
    _elevator.setNeutralMode(NeutralMode.Brake);

    _bottom_shooter.setInverted(true);

    //I guess its just a P system
    _PID_bot = new PIDController(ShootConstants.BOT_kP, 0, 0);
    _PID_top = new PIDController(ShootConstants.TOP_kP, 0, 0);

    rpmTop = s_tab.add("Top Shooter RPM", getTopRPM())
      .withPosition(1,0)
      .withSize(3,3)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();
    rpmTopText = s_tab.add("Top RPM", getTopRPM())
      .withPosition(0, 2)
      .withSize(1, 1)
      .getEntry();
    rpmBot = s_tab.add("Bottom Shooter RPM", getBotRPM())
      .withPosition(5,0)
      .withSize(3,3)
      .withWidget(BuiltInWidgets.kGraph)
      .getEntry();
    rpmBotText = s_tab.add("Bottom RPM", getBotRPM())
      .withPosition(4, 2)
      .withSize(1, 1)
      .getEntry();
    setpointTop = s_tab.add("Top Shooter setpoint", 0)
      .withPosition(1, 3)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
    setpointBot = s_tab.add("Bottom Shooter setpoint", 0)
      .withPosition(5, 3)
      .withSize(3, 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
    s_tab.add("Top PID", _PID_top)
      .withPosition(0, 0)
      .withSize(1,2);
    s_tab.add("Bottom PID", _PID_bot)
      .withPosition(4, 0)
      .withSize(1,2);


  }

  /**
   * Gets the rpm of the bottom shooter
   * @return The rpm of the bottom shooter
   */
  public double getBotRPM() {
    return (_bottom_encoder.getVelocity());
    
  }

  /**
   * Gets the rpm of the top shooter
   * @return The rpm of the top shooter
   */
  public double getTopRPM() {
    return (_top_encoder.getVelocity());
  }

  /**
   * Sets top shooter target rpm to val
   * @param val Target rpm
   */
  public void setTop(double val) {
    _PID_top.setSetpoint(val * 3.086);
  }
  /**
   * Sets top shooter target speed to whatever is on shuffleboard
   */
  public void setTop() {
    _PID_top.setSetpoint(_t_s * 3.086);
  }

  /**
   * Sets bottom shooter target rpm to val
   * @param val Target rpm
   */
  public void setBot(double val) {
    _PID_bot.setSetpoint(val * 3.104);
  }
  /**
   * Sets bottom shooter target speed to whatever is on shuffleboard
   */
  public void setBot() {
    _PID_bot.setSetpoint(_b_s * 3.104);
  }

  /**
   * Method to drive the ball into the shooter
   */
  public void driveBall() {
    _elevator.set(0.3);
  }

  public void stopBall() {
    _elevator.stopMotor();
  }

  //raw shoots only temporary
  public void rawShoot() {
    _top_shooter.set(.63);
    _bottom_shooter.set(.83);
  }

  public void stopShoot() {
    _top_shooter.stopMotor();
    _bottom_shooter.stopMotor();
  }

  /**
   * Calculates and sets the voltage on the motors
   */
  public void PIDLoop() {

    double top_out = _PID_top.calculate(getTopRPM());// + _feedforward_top.calculate(getTopRPM());
    //_top_shooter.set(top_out);
    _top_shooter.setVoltage(top_out);
    double bot_out = _PID_bot.calculate(getBotRPM()); //+ _feedforward_bot.calculate(getBotRPM());
    //_bottom_shooter.set(bot_out);
    _bottom_shooter.setVoltage(bot_out);
  }

  @Override
  public void periodic() {
    rpmTop.setNumber(getTopRPM());
    rpmTopText.setNumber(getTopRPM());
    rpmBot.setNumber(getBotRPM());
    rpmBotText.setNumber(getBotRPM());

    _t_s = setpointTop.getDouble(0);
    _b_s = setpointBot.getDouble(0);

    PIDLoop();
  }
}

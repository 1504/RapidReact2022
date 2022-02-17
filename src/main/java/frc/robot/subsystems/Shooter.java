// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ShootConstants;
import frc.robot.Utils.Utils;
import frc.robot.Utils.Presets;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Shooter extends SubsystemBase {

  private Presets _preset;

  private boolean _t;

  /*
  private final CANSparkMax _top_shooter;
  private final CANSparkMax _bottom_shooter;

  private final RelativeEncoder _top_encoder;
  private final RelativeEncoder _bottom_encoder;
  */

  private final WPI_TalonSRX _top_shooter;
  private final WPI_TalonSRX _bottom_shooter;  


  ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  
  private NetworkTableEntry _top_speed = m_tab.add("Top Shooter Speed", 1)
    .withPosition(0, 3)
    .withSize(2, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
  private NetworkTableEntry _bot_speed = m_tab.add("Bottom Shooter Speed", 1)
    .withPosition(2, 3)
    .withSize(2, 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();
  //private NetworkTableEntry _preset_chooser = m_tab.add("Shooter Presets", Presets.kMain)
    //.getEntry();


  public Shooter() {
    //_top_shooter = new CANSparkMax(ShootConstants.TOP_SHOOTER, MotorType.kBrushless);
    //_bottom_shooter = new CANSparkMax(ShootConstants.BOT_SHOOTER, MotorType.kBrushless);

    _top_shooter = new WPI_TalonSRX(ShootConstants.TOP_SHOOTER);
    _bottom_shooter = new WPI_TalonSRX(ShootConstants.BOT_SHOOTER);

    _preset = Presets.kMain;
    //maybe have testing enabled by controller. Will make it easier for builders
    _t = ShootConstants.TESTING;

    //_top_encoder = _top_shooter.getEncoder();
    //_bottom_encoder = _bottom_shooter.getEncoder();
/*
    m_tab.add("Top Encoder", _top_encoder.getVelocity()).withPosition(0, 2).withSize(1, 1);
    m_tab.add("Top Position", _top_encoder.getPosition()).withPosition(1, 2).withSize(1,1);
    m_tab.add("Bottom Encoder", _bottom_encoder.getVelocity()).withPosition(2, 2).withSize(1, 1);
    m_tab.add("Bottom Position", _bottom_encoder.getPosition()).withPosition(3, 2).withSize(1, 1);
*/

  }

  public void shoot() {
    double _t_s;
    double _b_s;
    if (_t) {
      _t_s = _top_speed.getDouble(1.0);
      _b_s = _bot_speed.getDouble(1.0);
    } else {
      _t_s = _preset.topVal;
      _b_s = _preset.botVal;
    }
    _top_shooter.set(_t_s);
    _bottom_shooter.set(_b_s);

  }

  public void stopMotors() {
    _top_shooter.stopMotor();
    _bottom_shooter.stopMotor();
  }

  public double test() {
    //return _bot_speed.getDouble(1.0);
    return 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

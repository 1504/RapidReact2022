// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.BuildConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MecanumDrivetrain extends SubsystemBase {
  
  //Motor Controllers
  private final CANSparkMax _front_left_motor;
  private final CANSparkMax _front_right_motor;
  private final CANSparkMax _back_right_motor;
  private final CANSparkMax _back_left_motor;

  //Encoders
  private final RelativeEncoder _front_left_encoder;
  private final RelativeEncoder _front_right_encoder;
  private final RelativeEncoder _back_right_encoder;
  private final RelativeEncoder _back_left_encoder;

  //Wheel locations
  private final Translation2d _front_left_location;
  private final Translation2d _front_right_location;
  private final Translation2d _back_right_location;
  private final Translation2d _back_left_location;

  private final AHRS _gyro;
  private final MecanumDriveKinematics _kinematics;
  private final MecanumDriveOdometry _odometry;

  ShuffleboardTab o_tab = Shuffleboard.getTab("Odometry");
  NetworkTableEntry position;

  public MecanumDrivetrain() {
    _front_left_motor = new CANSparkMax(DriveConstants.FRONT_LEFT, MotorType.kBrushless);
    _front_right_motor = new CANSparkMax(DriveConstants.FRONT_RIGHT, MotorType.kBrushless);
    _back_right_motor = new CANSparkMax(DriveConstants.BACK_RIGHT, MotorType.kBrushless);
    _back_left_motor = new CANSparkMax(DriveConstants.BACK_LEFT, MotorType.kBrushless);
    
    _front_left_encoder = _front_left_motor.getEncoder();
    _front_right_encoder = _front_right_motor.getEncoder();
    _back_left_encoder = _back_left_motor.getEncoder();
    _back_right_encoder = _back_right_motor.getEncoder();
    
    _front_left_location = new Translation2d(BuildConstants.WHEEL_TO_CENTER_SIDE_INCHES * BuildConstants.INCHES_TO_METERS, 
                                             BuildConstants.WHEEL_TO_CENTER_FRONT_INCHES * BuildConstants.INCHES_TO_METERS);
    _front_right_location = new Translation2d(BuildConstants.WHEEL_TO_CENTER_SIDE_INCHES * BuildConstants.INCHES_TO_METERS, 
                                             -BuildConstants.WHEEL_TO_CENTER_FRONT_INCHES * BuildConstants.INCHES_TO_METERS);
    _back_left_location = new Translation2d(-BuildConstants.WHEEL_TO_CENTER_SIDE_INCHES * BuildConstants.INCHES_TO_METERS, 
                                             BuildConstants.WHEEL_TO_CENTER_FRONT_INCHES * BuildConstants.INCHES_TO_METERS);
    _back_right_location = new Translation2d(-BuildConstants.WHEEL_TO_CENTER_SIDE_INCHES * BuildConstants.INCHES_TO_METERS, 
                                             -BuildConstants.WHEEL_TO_CENTER_FRONT_INCHES * BuildConstants.INCHES_TO_METERS);
  
    _kinematics = new MecanumDriveKinematics(_front_left_location, 
                                              _front_right_location, 
                                              _back_left_location, 
                                              _back_right_location);

    _gyro = new AHRS(SPI.Port.kMXP);

    _odometry = new MecanumDriveOdometry(_kinematics, _gyro.getRotation2d());
    _gyro.reset();
  
  }

  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
      _front_left_encoder.getVelocity(),
      _front_right_encoder.getVelocity(),
      _back_left_encoder.getVelocity(),
      _back_right_encoder.getVelocity()
    );
  }

  public void updateOdometry() {
    _odometry.update(_gyro.getRotation2d(), getCurrentState());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

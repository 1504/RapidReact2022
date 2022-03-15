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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
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

  //PIDs
  private final PIDController _front_left_PID;
  private final PIDController _front_right_PID;
  private final PIDController _back_left_PID;
  private final PIDController _back_right_PID;

  //Feed Forward
  private final SimpleMotorFeedforward _feedforward;

  private final AHRS _gyro;
  private final MecanumDriveKinematics _kinematics;
  private final MecanumDriveOdometry _odometry;
  private Pose2d _robot_position;

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
  
    _front_left_PID = new PIDController(0, 0, 0);
    _front_right_PID = new PIDController(0, 0, 0);
    _back_left_PID = new PIDController(0, 0, 0);
    _back_right_PID = new PIDController(0, 0, 0);

    _feedforward = new SimpleMotorFeedforward(0, 0);

    _kinematics = new MecanumDriveKinematics(_front_left_location, 
                                              _front_right_location, 
                                              _back_left_location, 
                                              _back_right_location);

    _gyro = new AHRS(SPI.Port.kMXP);

    _odometry = new MecanumDriveOdometry(_kinematics, _gyro.getRotation2d());
    _gyro.reset();
  
    position = o_tab.add("Position", _robot_position)
      .withPosition(0, 0)
      .getEntry();
  }

  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
      _front_left_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE,
      _front_right_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE,
      _back_left_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE,
      _back_right_encoder.getVelocity() / BuildConstants.GR * BuildConstants.WHEEL_CIRCUMFERENCE
    );
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    double flff = _feedforward.calculate(speeds.frontLeftMetersPerSecond);
    double frff = _feedforward.calculate(speeds.frontRightMetersPerSecond);
    double blff = _feedforward.calculate(speeds.rearLeftMetersPerSecond);
    double brff = _feedforward.calculate(speeds.rearRightMetersPerSecond);

    double fl = _front_left_PID.calculate(
      _front_left_encoder.getVelocity(), speeds.frontLeftMetersPerSecond
    );
    double fr = _front_right_PID.calculate(
      _front_right_encoder.getVelocity(), speeds.frontRightMetersPerSecond
    );
    double bl = _back_left_PID.calculate(
      _back_left_encoder.getVelocity(), speeds.rearLeftMetersPerSecond
    );
    double br = _back_right_PID.calculate(
      _back_right_encoder.getVelocity(), speeds.rearRightMetersPerSecond
    );

    
  }

  public Pose2d updateOdometry() {

    return _odometry.update(_gyro.getRotation2d(), getCurrentState());
  }

  public Pose2d getPose() {
    return _odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    _odometry.resetPosition(pose, _gyro.getRotation2d());
  }

  public double getHeading() {
    return _gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -_gyro.getRate();
  }

  public MecanumDriveKinematics getKinematics() {
    return _kinematics;
  }

  public SimpleMotorFeedforward getFeedForward() {
    return _feedforward;
  }

  @Override
  public void periodic() {
    _robot_position = updateOdometry();
    position.setValue(_robot_position);
  }
}

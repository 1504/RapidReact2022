// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  
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

  //Drivetrain
  private final MecanumDrive _drive;

  //Translations for kinematic drive later
  private final Translation2d k_front_left_location;
  private final Translation2d k_front_right_location;
  private final Translation2d k_back_right_location;
  private final Translation2d k_back_left_location;

  //Kinematic drive for later
  private final MecanumDriveKinematics _kinematics;

  //PID drive
  /*
  private final SparkMaxPIDController fl_pid_controller;
  private final SparkMaxPIDController fr_pid_controller;
  private final SparkMaxPIDController br_pid_controller;
  private final SparkMaxPIDController bl_pid_controller;
*/
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  //Gyro
  //private final AHRS _gyro;

  //Shuffleboard tab
  ShuffleboardTab m_tab = Shuffleboard.getTab("Main");

  public Drivetrain() {
    _front_left_motor = new CANSparkMax(DriveConstants.FRONT_LEFT, MotorType.kBrushless);
    _front_right_motor = new CANSparkMax(DriveConstants.FRONT_RIGHT, MotorType.kBrushless);
    _back_right_motor = new CANSparkMax(DriveConstants.BACK_RIGHT, MotorType.kBrushless);
    _back_left_motor = new CANSparkMax(DriveConstants.BACK_LEFT, MotorType.kBrushless);

    _drive = new MecanumDrive(_front_left_motor, _back_left_motor, _front_right_motor, _back_right_motor);

    //Dummy numbers
    k_front_left_location = new Translation2d(1, 1);
    k_front_right_location = new Translation2d(1, 1);
    k_back_right_location = new Translation2d(1, 1);
    k_back_left_location = new Translation2d(1, 1);

    _kinematics = new MecanumDriveKinematics(k_front_left_location, k_front_right_location, k_back_left_location, k_back_right_location);

    //Encoder stuff
    _front_left_encoder = _front_left_motor.getEncoder();
    _front_right_encoder = _front_right_motor.getEncoder();
    _back_left_encoder = _back_left_motor.getEncoder();
    _back_right_encoder = _back_right_motor.getEncoder();
    

    //_gyro = new AHRS(SPI.Port.kMXP);

    //PID defining
    /*
    fl_pid_controller = _front_left_motor.getPIDController();
    fl_pid_controller.setFeedbackDevice(_front_left_encoder);
    fr_pid_controller = _front_right_motor.getPIDController();
    fr_pid_controller.setFeedbackDevice(_front_right_encoder);
    bl_pid_controller = _back_left_motor.getPIDController();
    bl_pid_controller.setFeedbackDevice(_back_left_encoder);
    br_pid_controller = _back_right_motor.getPIDController();
    br_pid_controller.setFeedbackDevice(_back_right_encoder);
    */

    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    //Shuffleboard stuff
    m_tab.add("Drive", _drive)
      .withPosition(1, 0)
      .withSize(4, 2);
    m_tab.add("Front Left", _front_left_encoder.getVelocity())
      .withPosition(0, 0)
      .withSize(1, 1);
    m_tab.add("Back Left", _back_left_encoder.getVelocity())
      .withPosition(0, 1)
      .withSize(1, 1);
    m_tab.add("Front Right", _front_right_encoder.getVelocity())
      .withPosition(5, 0)
      .withSize(1, 1);
    m_tab.add("Back Right", _back_right_encoder.getVelocity())
      .withPosition(5, 1)
      .withSize(1, 1);
  }

  /**
   * main method to drive the robot
   * @param ySpeed The robot speed in the y axis (front/back) values from -1 to 1
   * @param xSpeed The robot speed in the x axis (left/right) values from -1 to 1
   * @param zRot The robot rotation speed values from -1 to 1
   */
  public void cartesianDrive(double ySpeed, double xSpeed, double zRot) {
    _drive.driveCartesian(ySpeed, xSpeed, zRot);
  }
  /**
   * main method to drive the robot with gyro
   * @param ySpeed The robot speed in the y axis (front/back) values from -1 to 1
   * @param xSpeed The robot speed in the x axis (left/right) values from -1 to 1
   * @param zRot The robot rotation speed values from -1 to 1
   * @param gyroAngle the gyro angle
   */
  public void cartesianDrive(double ySpeed, double xSpeed, double zRot, double gyroAngle) {
    _drive.driveCartesian(ySpeed, xSpeed, zRot, gyroAngle);
  }

  /**
   * Method to stop driving
   */
  public void stopDrive() {
    _drive.stopMotor();
  }

  @Override
  public void periodic() {
    //m_tab.addNumber("Encoder Distance", () -> fl_encoder.getDistance());
    //m_tab.addNumber("Encoder Rate", () -> fl_encoder.getRate());
  }
}

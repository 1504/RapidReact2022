// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
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

  //Drivetrain
  private final MecanumDrive _drive;

  //Translations for kinematic drive later
  private final Translation2d k_front_left_location;
  private final Translation2d k_front_right_location;
  private final Translation2d k_back_right_location;
  private final Translation2d k_back_left_location;

  //Kinematic drive for later
  private final MecanumDriveKinematics _kinematics;

  //Encoders
  private final Encoder fl_encoder;

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
    fl_encoder = new Encoder(0, 1);
    /*
    fl_encoder.setSamplesToAverage(5);
    fl_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
    fl_encoder.setMinRate(1.0);
    */
    

    //_gyro = new AHRS(SPI.Port.kMXP);

    //Shuffleboard stuff
    m_tab.add("Drive", _drive).withPosition(0, 0).withSize(3, 2);
    //ShuffleboardLayout encoders = m_tab.getLayout("List Layout", "Encoders").withPosition(3, 1).withSize(2, 2);
    //encoders.add("Left Encoder", fl_encoder);
    m_tab.add("Encoders", fl_encoder).withPosition(3, 1).withSize(2,2);
  }

  public void cartesianDrive(double ySpeed, double xSpeed, double zRot) {
    _drive.driveCartesian(ySpeed, xSpeed, zRot);
  }

  public void cartesianDrive(double ySpeed, double xSpeed, double zRot, double gyroAngle) {
    _drive.driveCartesian(ySpeed, xSpeed, zRot, gyroAngle);
  }

  public void stopDrive() {
    _drive.stopMotor();
  }


  @Override
  public void periodic() {
    //m_tab.addNumber("Encoder Distance", () -> fl_encoder.getDistance());
    //m_tab.addNumber("Encoder Rate", () -> fl_encoder.getRate());
  }
}

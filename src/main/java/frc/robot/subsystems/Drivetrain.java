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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  
  private final CANSparkMax _front_left_motor;
  private final CANSparkMax _front_right_motor;
  private final CANSparkMax _back_right_motor;
  private final CANSparkMax _back_left_motor;

  private final MecanumDrive _drive;

  private final Translation2d k_front_left_location;
  private final Translation2d k_front_right_location;
  private final Translation2d k_back_right_location;
  private final Translation2d k_back_left_location;

  private final MecanumDriveKinematics _kinematics;

  //private final Encoder 

  //private final AHRS _gyro;

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


    //_gyro = new AHRS(SPI.Port.kMXP);

    ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
    m_tab.add("Drive", _drive).withPosition(0, 0).withSize(3, 2);
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
    // This method will be called once per scheduler run
  }
}

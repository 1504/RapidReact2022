// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IOConstants;

//Lights will be utilized on the robot to indicate states
public class Lights extends SubsystemBase {
  
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_led_buffer;

  public Lights() {
    m_led = new AddressableLED(IOConstants.LED_PORT);
    m_led_buffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_led_buffer.getLength());

    m_led.setData(m_led_buffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

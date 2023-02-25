// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private AddressableLED addressableLED = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(59);
  // private Spark m_blinkin = new Spark(0);

  /* Creates a new LED class. */
  public LED() {
    addressableLED.setLength(buffer.getLength());
    setRed();
    addressableLED.start();

    var isRed = NetworkTableInstance
      .getDefault()
      .getTable("FMSInfo")
      .getEntry("IsRedAlliance")
      .getBoolean(true);

    if (isRed == true){
      // m_blinkin.set(-0.01);
      System.out.println("led RED");
    } else {
      // m_blinkin.set(0.19);
      System.out.println("led BLUE");
    }
  }

  public void setRed() {
    setSolidColor(255, 0, 0);
  }

  public void setBlue() {
    setSolidColor(0, 0, 255);
  }

  public void setGreen() {
    setSolidColor(0, 255, 0);
  }

  private void setSolidColor(int r, int g, int b) {
    for (var i = 0; i < buffer.getLength(); ++i) {
      buffer.setRGB(i, r, g, b);
    }
    addressableLED.setData(buffer);
  }

  // public void set(double val) {
  //   if ((val >= -1.0) && (val <= 1.0)) {
  //     m_blinkin.set(val);
  //   }
  // }

  // public void solidOrange() {
  //   set(0.65);
  // }

  // public void solidYellow() {
  //   m_blinkin.set(0.69);
  // }

  // public void solidPurple() {
  //   set(0.91);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

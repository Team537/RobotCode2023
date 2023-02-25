// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {

  private static Spark m_blinkin = null;

  /** Creates a new LED class. */
  public void blinkin(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
    solidOrange();
  }

  /* Creates a new LED class. */
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public void solidOrange() {
    set(0.65);
  }

  public void solidYellow() {
    set(0.69);
  }

  public void solidPurple() {
    set(0.91);
  }

  /* Creates a new LED class. */
  public LED() {

    boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
    if (isRed == true){
      m_blinkin.set(-0.01);
      System.out.println("led RED");
    } else {
      m_blinkin.set(0.19);
      System.out.println("led BLUE");
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

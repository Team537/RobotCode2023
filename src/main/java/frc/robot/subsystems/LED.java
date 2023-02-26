// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  private AddressableLED addressableLED = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.length);
  private boolean fallen = false;
  private boolean low_goal = false;
  private boolean mid_goal = false;
  private boolean high_goal = false;
  private boolean shelf = false;
  private boolean intaking = false;
  private boolean outaking = false;
  private boolean cube = false;
  private boolean cone = false;
  private LedMode mode = LedMode.DISABLED_NEUTRAL;
  private Alliance alliance = Alliance.Invalid;
  private String ledState = "Default";
  // private Spark m_blinkin = new Spark(0);
 
  public void update() {
    // Update alliance color
    
      alliance = DriverStation.getAlliance();
    
    if (DriverStation.isDisabled()) {
      switch (alliance) {
        case Red:
          mode = LedMode.DISABLED_RED;
          ledState = "Disabeld Red";
          break;
        case Blue:
          mode = LedMode.DISABLED_BLUE;
          ledState = "Disabeld Blue";
          break;
        default:
          mode = LedMode.DISABLED_NEUTRAL;
          ledState = "Disabeld Neutral";
          break;
      } 
    } else if (fallen) {
      mode = LedMode.FALLEN;
      ledState = "Fallen";
    } 
    else if (cone) {
      mode = LedMode.CONE;
      ledState = "Cone";
    } 
    else if (cube) {
      mode = LedMode.CUBE;
      ledState = "Cube";
    } 
    else if (outaking && DriverStation.isTeleop()) {
      mode = LedMode.OUTAKING;
      ledState = "Outaking";
    } 
    else if (intaking && DriverStation.isTeleop()) {
      mode = LedMode.INTAKING;
      ledState = "Intaking";
    } 
    else if (low_goal && DriverStation.isTeleop()) {
      mode = LedMode.LOW_GOAL;
      ledState = "Low Goal";
    } 
    else if (mid_goal && DriverStation.isTeleop()) {
      mode = LedMode.MID_GOAL;
      ledState = "Mid Goal";
    } 
    else if (high_goal && DriverStation.isTeleop()) {
      mode = LedMode.HIGH_GOAL;
      ledState = "High Goal";
    } else if (shelf && DriverStation.isTeleop()) {
      mode = LedMode.SHELF;
      ledState = "Shelf";
    } else { switch (alliance) {
      case Red:
        mode = LedMode.DEFAULT_TELEOP_RED;
        ledState = "Default Teleop Red";
        break;
      case Blue:
        mode = LedMode.DEFAULT_TELEOP_BLUE;
        ledState = "Default Teleop Blue";
        break;
      default:
        mode = LedMode.DISABLED_NEUTRAL;
        ledState = "Default";
        break;
    } 
      
    }
  }
  /* Creates a new LED class. */
  public LED() {
    addressableLED.setLength(buffer.getLength());
    // setRed();
    addressableLED.start();
   
    // var isRed = NetworkTableInstance
    //   .getDefault()
    //   .getTable("FMSInfo")
    //   .getEntry("IsRedAlliance")
    //   .getBoolean(true);

  //   if (isRed == true){
  //     // m_blinkin.set(-0.01);
  //     System.out.println("led RED");
  //   } else {
  //     // m_blinkin.set(0.19);
  //     System.out.println("led BLUE");
  //   }
  // }

  // public void setRed() {
  //   setSolidColor(255, 0, 0);
  // }

  // public void setBlue() {
  //   setSolidColor(0, 0, 255);
  // }

  // public void setGreen() {
  //   setSolidColor(0, 255, 0);
  }
  public static enum LedMode {
    FALLEN, 
    LOW_GOAL, 
    MID_GOAL, 
    HIGH_GOAL,
    SHELF, 
    CONE,
    CUBE,  
    INTAKING,
    OUTAKING, 
    DEFAULT_TELEOP_RED, 
    DEFAULT_TELEOP_BLUE,
    DISABLED_RED, 
    DISABLED_BLUE, 
    DISABLED_NEUTRAL, 
  }
  public void setMode(LedMode mode, boolean sameBattery) {
    switch (mode) {
      case FALLEN:
        strobe(Color.kWhite);
        break;
      case INTAKING:
        setSolidColor(Color.kGold);
        break;
      case OUTAKING:
        setSolidColor(Color.kFirebrick);
        break;
      case LOW_GOAL:
        breath(Color.kRed, Color.kFloralWhite);
        break;
      case MID_GOAL:
        breath(Color.kRed, Color.kAzure);
        break;
      case HIGH_GOAL:
        breath(Color.kRed, Color.kDarkGoldenrod);
        break;
     case SHELF:
        breath(Color.kRed, Color.kCrimson);
        break;
      case DEFAULT_TELEOP_RED:
      wave(Color.kRed, Color.kBlack, LEDConstants.waveAllianceFullLength,
      LEDConstants. waveAllianceDuration);
      case CONE:
      setSolidColor(Color.kYellow);
        break;
      case CUBE:
      setSolidColor(Color.kPurple);
        break;
      case DEFAULT_TELEOP_BLUE:
      wave(Color.kBlue, Color.kBlack, LEDConstants.waveAllianceFullLength,
      LEDConstants. waveAllianceDuration);
        break;
      case DISABLED_RED:
      setSolidColor(Color.kRed);
        break;
      case DISABLED_BLUE:
      setSolidColor(Color.kBlue);
        break;
      default:
        setSolidColor(Color.kBlack);
       break;
    }
       
    }

  private void setSolidColor(Color color) {
    for (var i = 0; i < buffer.getLength(); ++i) {
      buffer.setLED(i, color);
    }
    addressableLED.setData(buffer);
  }
  private void strobe(Color color) {
    boolean on =
        ((Timer.getFPGATimestamp() % LEDConstants.strobeDuration) / LEDConstants.strobeDuration) > 0.5;
    setSolidColor(on ? color : Color.kBlack);
  }

  private void breath(Color c1, Color c2) {
    double x = ((Timer.getFPGATimestamp() % LEDConstants.breathDuration) / LEDConstants.breathDuration)
        * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    setSolidColor(new Color(red, green, blue));
  }
  private void setLedsSymmetrical(int index, Color color) {
    buffer.setLED((LEDConstants.centerLed + index) % LEDConstants.length, color);
    buffer.setLED(LEDConstants.centerLed - index, color);
  }
  private void rainbow(double fullLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / fullLength;
    for (int i = 0; i < LEDConstants.halfLength; i++) {
      x += xDiffPerLed;
      x %= 180.0;
      setLedsSymmetrical(i, Color.fromHSV((int) x, 255, 255));
    }
  }
  
  private void wave(Color c1, Color c2, double fullLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0
        * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / fullLength;
    for (int i = 0; i < LEDConstants.halfLength; i++) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI),LEDConstants. waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      setLedsSymmetrical(i, new Color(red, green, blue));
    }
    
  }
  public void setIntaking(boolean active) {
    intaking = active;
  }
  public void setOutaking(boolean active) {
    outaking = active;
  }
  public void setLowGoal(boolean active) {
    low_goal = active;
  }
  public void setMidGoal(boolean active) {
    mid_goal = active;
  }
  public void setHighGoal(boolean active) {
    high_goal = active;
  }
  public void setShelf(boolean active) {
    shelf = active;
  }
  public void setCube(boolean active) {
    cube = active;
  }
  public void setCone(boolean active) {
    cone = active;
  }
  public void setFallen(boolean active) {
    fallen = active;
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
    SmartDashboard.putString("Led State", ledState);
    // This method will be called once per scheduler run
  }
}

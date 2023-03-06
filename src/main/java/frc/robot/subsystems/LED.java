// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private static Spark m_blinkin = new Spark(0);

  // private AddressableLED addressableLED = new AddressableLED(1);
  // private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.length);
  private boolean fallen = false;
  private boolean low_goal = false;
  private boolean mid_goal = false;
  private boolean high_goal = false;
  private boolean shelf = false;
  private boolean intaking = false;
  private boolean fastOutaking = false;
  private boolean outaking = false;
  private ManipulatorObject manipulatorObject = null;
  private boolean boost_driving = false;
  private boolean driving = false;
  private boolean auto_start = false;
  private boolean auto_end = false;

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
    else if (manipulatorObject == ManipulatorObject.CONE) {
      mode = LedMode.CONE;
      ledState = "Cone";
    }
    else if (manipulatorObject == ManipulatorObject.CUBE) {
      mode = LedMode.CUBE;
      ledState = "Cube";
    } 
    else if (boost_driving) {
      mode = LedMode.BOOST_DRIVING;
      ledState = "Boost Driving";
    } 
    else if (driving) {
      mode = LedMode.DRIVING;
      ledState = "Driving";
    } 
    else if (outaking && DriverStation.isTeleop()) {
      mode = LedMode.OUTAKING;
      ledState = "Outaking";
    } 
    else if (intaking && DriverStation.isTeleop()) {
      mode = LedMode.INTAKING;
      ledState = "Intaking";
    } 
    else if (fastOutaking && DriverStation.isTeleop()) {
      mode = LedMode.FAST_OUTAKING;
      ledState = "Fast Intaking";
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
    } else if (auto_start) {
      mode = LedMode.AUTO_START;
      ledState = "Auto Start";
    } else if (auto_end) {
      mode = LedMode.AUTO_END;
      ledState = "Auto End";
    }
    else { switch (alliance) {
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

   if(DriverStation.isTeleop()){
       auto_end = false;
       auto_start = false;

   }

    setMode(mode);

  }
  /* Creates a new LED class. */
  // public LED() {

    //addressableLED.setLength(buffer.getLength());
    // setRed();
    //addressableLED.start();
   
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
  // }

  // }

  public static enum LedMode {
    FALLEN, 
    LOW_GOAL, 
    MID_GOAL, 
    HIGH_GOAL,
    SHELF, 
    CONE,
    CUBE,  
    INTAKING,
    FAST_OUTAKING,
    OUTAKING, 
    DEFAULT_TELEOP_RED, 
    DEFAULT_TELEOP_BLUE,
    DISABLED_RED, 
    BOOST_DRIVING,
    DRIVING,
    DISABLED_BLUE, 
    DISABLED_NEUTRAL, 
    AUTO_START,
    AUTO_END,
  }

  public static enum ManipulatorObject {
    CONE,
    CUBE
  }

    // Sets the LED Blinkin based on the current MODE
  public void setMode(LedMode mode) {
    switch (mode) {

      case FALLEN:
        // strobe(Color.kWhite);
        m_blinkin.set(-0.05);
        break;

      case INTAKING:
        // setSolidColor(Color.kGold);
        m_blinkin.set(0.67);
        break;

      case FAST_OUTAKING:
        // setSolidColor(Color.kGold);
        m_blinkin.set(-0.73);
        break;

      case OUTAKING:
        // setSolidColor(Color.kFirebrick);
        m_blinkin.set(-0.59);
        break;

      case LOW_GOAL:
        // breath(Color.kRed, Color.kFloralWhite);
        m_blinkin.set(-0.17);
        break;

      case MID_GOAL:
        // breath(Color.kRed, Color.kAzure);
        m_blinkin.set(-0.39);
        break;

      case HIGH_GOAL:
        // breath(Color.kRed, Color.kDarkGoldenrod);
        m_blinkin.set(-0.25);
        break;

     case SHELF:
        // breath(Color.kRed, Color.kCrimson);
        m_blinkin.set(-0.25);
        break;

     case BOOST_DRIVING:
        // wave(Color.kGreen, Color.kBlack, LEDConstants.waveAllianceFullLength, LEDConstants. waveAllianceDuration);
        m_blinkin.set(-0.37);
        break;

      case DRIVING:
        // wave(Color.kLime, Color.kBlack, LEDConstants.waveAllianceFullLength, LEDConstants. waveAllianceDuration);
        m_blinkin.set(-0.49);
        break;

      case DEFAULT_TELEOP_RED:
      // wave(Color.kRed, Color.kBlack, LEDConstants.waveAllianceFullLength,
      // LEDConstants. waveAllianceDuration);
        m_blinkin.set(-0.25);
        break;

      case CONE:
      // setSolidColor(Color.kYellow);
        m_blinkin.set(0.69);
        break;

      case CUBE:
      // setSolidColor(Color.kPurple);
        m_blinkin.set(0.91);
        break;

        case AUTO_START:
        // setSolidColor(Color.kBlue);
          m_blinkin.set(-0.91);
          break;

      case AUTO_END:
        // setSolidColor(Color.kBlue);
          m_blinkin.set(-0.87);
          break;

      case DEFAULT_TELEOP_BLUE:
      // wave(Color.kBlue, Color.kBlack, LEDConstants.waveAllianceFullLength,
      // LEDConstants. waveAllianceDuration);
        m_blinkin.set(-0.15);
        break;

      case DISABLED_RED:
      // setSolidColor(Color.kRed);
        m_blinkin.set(-0.17);
        break;

      case DISABLED_BLUE:
      // setSolidColor(Color.kBlue);
        m_blinkin.set(-0.15);
        break;

    
        
      default:
        // setSolidColor(Color.kBlack);
        m_blinkin.set(0.99);
       break;
    }
       
    }

  // private void setSolidColor(Color color) {
  //   // for (var i = 0; i < buffer.getLength(); ++i) {
  //   //   buffer.setLED(i, color);
  //   // }
  //   // addressableLED.setData(buffer);
  // }

  // private void strobe(Color color) {
  //   boolean on =
  //       ((Timer.getFPGATimestamp() % LEDConstants.strobeDuration) / LEDConstants.strobeDuration) > 0.5;
  //   setSolidColor(on ? color : Color.kBlack);
  // }

  // private void breath(Color c1, Color c2) {
  //   double x = ((Timer.getFPGATimestamp() % LEDConstants.breathDuration) / LEDConstants.breathDuration)
  //       * 2.0 * Math.PI;
  //   double ratio = (Math.sin(x) + 1.0) / 2.0;
  //   double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
  //   double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
  //   double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
  //   setSolidColor(new Color(red, green, blue));
  // }

  // private void setLedsSymmetrical(int index, Color color) {
  //   // buffer.setLED((LEDConstants.centerLed + index) % LEDConstants.length, color);
  //   // buffer.setLED(LEDConstants.centerLed - index, color);
  // }
  
  // private void rainbow(double fullLength, double duration) {
  //   double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
  //   double xDiffPerLed = 180.0 / fullLength;
  //   for (int i = 0; i < LEDConstants.halfLength; i++) {
  //     x += xDiffPerLed;
  //     x %= 180.0;
  //     setLedsSymmetrical(i, Color.fromHSV((int) x, 255, 255));
  //   }
  // }
  
  // private void wave(Color c1, Color c2, double fullLength, double duration) {
  //   double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0
  //       * Math.PI;
  //   double xDiffPerLed = (2.0 * Math.PI) / fullLength;
  //   for (int i = 0; i < LEDConstants.halfLength; i++) {
  //     x += xDiffPerLed;
  //     double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0) / 2.0;
  //     if (Double.isNaN(ratio)) {
  //       ratio = (-Math.pow(Math.sin(x + Math.PI),LEDConstants. waveExponent) + 1.0) / 2.0;
  //     }
  //     if (Double.isNaN(ratio)) {
  //       ratio = 0.5;
  //     }
  //     double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
  //     double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
  //     double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
  //     setLedsSymmetrical(i, new Color(red, green, blue));
  //   }
    
  // }

  // Sets the variable in this file active based on another file's variable ( trace the method )
  // public void setIntaking(boolean active) {
  //   intaking = active;
  // }
  // public void setOutaking(boolean active) {
  //   outaking = active;
  // }

  public void toggleOutake() {
    outaking = !outaking;
  }
  public void toggleIntake() {
    intaking = !intaking;
  }
  public void toggleFastOutake() {
    fastOutaking = !fastOutaking;
  }
  public void autoStart() {
    auto_start = true;
  }

  public void autoEnd() {
    auto_start = false;
    auto_end = true;
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
  public void toggleCone() {
    manipulatorObject = manipulatorObject == ManipulatorObject.CONE
      ? null
      : ManipulatorObject.CONE;
  }
  public void toggleCube() {
    manipulatorObject = manipulatorObject == ManipulatorObject.CUBE
      ? null
      : ManipulatorObject.CUBE;
  }
  public void setFallen(boolean active) {
    fallen = active;
  }
  public void setBoostDriving(boolean active) {
    boost_driving = active;
  }
  public void setDriving(boolean active) {
    driving = active;
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

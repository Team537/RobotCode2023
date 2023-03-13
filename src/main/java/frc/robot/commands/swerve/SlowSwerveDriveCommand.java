/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.Constants.DriveSpeedConstants;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class SlowSwerveDriveCommand extends CommandBase {
  
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_driveInput, m_strafeInput, m_rotationInput, m_triggerInput;
  private final boolean m_isFieldRelative;
  private final LED m_LED;

  /**
   * Creates a new ExampleCommand.
   *
   * 
   */
  public SlowSwerveDriveCommand(DriveSubsystem m_drive, DoubleSupplier driveInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier triggerInput, boolean isFieldRelative, LED m_LED) {
    this.m_drive = m_drive;
    m_driveInput = driveInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_isFieldRelative = isFieldRelative;
    m_triggerInput = triggerInput;
    this.m_LED = m_LED;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print( " \n Slow Drive Started");
  }

  public double calculateDrivePercentage(double JoystickPercent, double TriggerPercent) {
    double OutputSpeedPercent = 0.0;
    
                                  //deadband
    if(Math.abs(JoystickPercent)> 0.05) {
      if(JoystickPercent < 0) {
        TriggerPercent *= -1;
      }

      OutputSpeedPercent = (JoystickPercent * DriveSpeedConstants.kBaseRobotSpeed) + (JoystickPercent * (TriggerPercent * (DriveSpeedConstants.kMaxRobotSpeed - DriveSpeedConstants.kMaxRobotSpeed)));
    }
    else {
      //when it is lower than the deadband, it sets it to zero
      OutputSpeedPercent = 0;
    }

    return OutputSpeedPercent;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
                                                      // 0.05 are the joystick deadbands. if the joystick is less than that value, is makes it equal to 0 
    double drive =   calculateDrivePercentage(m_driveInput.getAsDouble(), m_triggerInput.getAsDouble());
    double strafe =  calculateDrivePercentage(m_strafeInput.getAsDouble(), m_triggerInput.getAsDouble());
    double rotation = Math.abs(m_rotationInput.getAsDouble()) > 0.05 ? m_rotationInput.getAsDouble() : 0;


    m_drive.slowDrive(drive, strafe, rotation, m_isFieldRelative, true);    // Forward/Back drive, Left/Right Strafe, Left/Right Turn
    if(Math.abs(m_drive.getVelocity()) > 0 && m_drive.driveState.equals("Drive") && DriverStation.isTeleop()) {
      m_LED.setSlowDriving(true);
      m_LED.setDriving(false);
      
    } else{
      m_LED.setSlowDriving(false);

    }

    //detects pitch and roll from the start position
    if(Math.abs(m_drive.getPitch()) > 75 || Math.abs(m_drive.getRoll()) > 75) {
      m_LED.setFallen(true);
    } else{
      m_LED.setFallen(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print(" \n Slow Drive Started");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

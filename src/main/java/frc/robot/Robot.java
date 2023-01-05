// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  String trajectoryJSON1 =  "paths/Test1.wpilib.json";
  String trajectoryJSON2 =  "paths/Test 2.wpilib.json";

     
  private RobotContainer m_robotContainer;

  Trajectory trajectory = new Trajectory();
  Trajectory trajectory1 = new Trajectory();
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String test1 = "Test1";
  private static final String test2 = "Test2";
  private String m_autoSelected;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  

    try {
      Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON1, ex.getStackTrace());
   }

   try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
    trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
 } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON2, ex.getStackTrace());
 }

 m_chooser.addOption("Test 1", test1);
 m_chooser.addOption("Test 2", test2);

 SmartDashboard.putData(m_chooser);
    
  
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
   
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance()); 
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

   
  }

  @Override
  public void disabledPeriodic() {

  
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {


    m_autoSelected = m_chooser.getSelected();


    switch(m_autoSelected){

      case(test1):
        m_autonomousCommand = m_robotContainer.getAutonomousCommand(trajectory);
      break;

      case(test2):
        m_autonomousCommand = m_robotContainer.getAutonomousCommand(trajectory1);
      break;
    }
 
  


 if (m_autonomousCommand != null) {
  m_autonomousCommand.schedule();
}
  }
  

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    
  }

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
   
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

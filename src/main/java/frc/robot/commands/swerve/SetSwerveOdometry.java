package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;


/** Sets the robot's position */
public class SetSwerveOdometry extends CommandBase {
  
  private final DriveSubsystem m_drive;

  private final FieldSim m_fieldSim;
  private Pose2d m_pose2d;

  /**
   * Sets the robot's position
   *
   * @param swerveDrive Swerve's odometry is set
   * @param pose2d position to set odometry to
   */
  
  /**
   * Sets the robot's position
   *
   * @param swerveDrive Swerve's odometry is set
   * @param pose2d position to set odometry to
   * @param fieldSim fieldSim to set robot's position if we're simulating the robot
   */

   //sets the position of the gyro
  public SetSwerveOdometry(DriveSubsystem m_drive, Pose2d pose2d, FieldSim fieldSim) {
    if (RobotBase.isSimulation() && fieldSim == null)
      System.out.println(
          "SetOdometry Command Error: Robot is in Simulation, but you did not add FieldSim to the argument");

    this.m_drive = m_drive;
    m_fieldSim = fieldSim;
    m_pose2d = pose2d;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setOdometry(m_pose2d);
    if (RobotBase.isSimulation()) m_fieldSim.resetRobotPose(m_pose2d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
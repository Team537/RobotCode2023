package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


                  //resets the gyro angle of the robot
public class ResetOdometry extends CommandBase {
  /** Creates a new ResetGyro. */
  private final DriveSubsystem m_drive;

  public ResetOdometry(DriveSubsystem m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive =m_drive;
    addRequirements( m_drive);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetGyro();
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
package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/** Sets the drivetrain to neutral (coast/brake) */
public class SetSwerveBrakeMode extends CommandBase {
  
  private final DriveSubsystem m_drive;
  private final NeutralMode m_mode;

  /**
   * Sets the drivetrain neutral mode (coast/brake).
   *
   * @param driveTrain The driveTrain used by this command.
   * @param mode {@link DriveTrainNeutralMode}: COAST, BRAKE, or HALF_BRAKE.
   */
  public SetSwerveBrakeMode( DriveSubsystem m_drive, NeutralMode mode) {
    this.m_drive = m_drive;
    m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements( m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setBrakeMode(m_mode);
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
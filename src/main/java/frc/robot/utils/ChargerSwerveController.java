package frc.robot.utils;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

public class ChargerSwerveController {
    private Pose2d m_poseError = new Pose2d();
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();
    private boolean m_enabled = true;
  
    private PIDController m_xController;
    private PIDController m_yController;
    private PIDController m_thetaController;
  
    /**
     * Custom Swerve Drive controller
     */
    
    public ChargerSwerveController(
        PIDController xController, PIDController yController, PIDController thetaController) {
      m_xController = xController;
      m_yController = yController;
      m_thetaController = thetaController;
      m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }
  
    
    public boolean atReference() {
      final var eTranslate = m_poseError.getTranslation();
      final var eRotate = m_rotationError;
      final var tolTranslate = m_poseTolerance.getTranslation();
      final var tolRotate = m_poseTolerance.getRotation();
      return Math.abs(eTranslate.getX()) < tolTranslate.getX()
          && Math.abs(eTranslate.getY()) < tolTranslate.getY()
          && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }
  
    public void setTolerance(Pose2d tolerance) {
      m_poseTolerance = tolerance;
    }
  
   
    @SuppressWarnings("LocalVariableName")
    public ChassisSpeeds calculate(
        Pose2d currentPose,
        Pose2d poseRef,
        double linearVelocityRefMeters,
        Rotation2d angleRef,
        double angleVelocityRefRadians) {
  
      // Calculate velocities (field-relative).
      double xFF = linearVelocityRefMeters * poseRef.getRotation().getCos();
      double yFF = linearVelocityRefMeters * poseRef.getRotation().getSin();
      double thetaFF = angleVelocityRefRadians;
  
      m_poseError = poseRef.relativeTo(currentPose);
      m_rotationError = angleRef.minus(currentPose.getRotation());
  
      if (!m_enabled) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
      }
  
      double xFeedback = m_xController.calculate(currentPose.getX(), poseRef.getX());
      double yFeedback = m_yController.calculate(currentPose.getY(), poseRef.getY());
      double thetaFeedback =
          m_thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());
  
    
      return ChassisSpeeds.fromFieldRelativeSpeeds(
          xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback, currentPose.getRotation());
    }
  
  
    public ChassisSpeeds calculate(
        Pose2d currentPose,
        Trajectory.State driveState,
        RotationSequence.State holonomicRotationState) {
      return calculate(
          currentPose,
          driveState.poseMeters,
          driveState.velocityMetersPerSecond,
          holonomicRotationState.position,
          holonomicRotationState.velocityRadiansPerSec);
    }
  
    /**
     * Enables and disables the controller for troubleshooting problems. 
     */
    public void setEnabled(boolean enabled) {
      m_enabled = enabled;
    }
}

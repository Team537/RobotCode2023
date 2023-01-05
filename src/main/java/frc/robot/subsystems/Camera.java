package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {

    PhotonCamera camera = new PhotonCamera("USB Camera 0");
    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    boolean hasTargets = result.hasTargets();
    
public void camera(){


SmartDashboard.putBoolean("Target Spotted", hasTargets);
SmartDashboard.putNumber("Target ID", target.getFiducialId());
SmartDashboard.putNumber("Target Yaw", target.getYaw());
SmartDashboard.putNumber("Target Pitch", target.getPitch());
SmartDashboard.putNumber("Target Skew", target.getSkew());
SmartDashboard.putNumber("Target Area", target.getArea());   


}

    public PhotonPipelineResult getLatestResult(){

    return camera.getLatestResult();

    }
    
    
}

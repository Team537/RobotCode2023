package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {

    
    boolean m_LimelightHasValidTarget = false;
    
public void camera(){

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if (tv < 1.0)
    {
        m_LimelightHasValidTarget = false;
    }else{
        m_LimelightHasValidTarget = true;
    }

    SmartDashboard.putBoolean("Is there a target: ", m_LimelightHasValidTarget);
}

    
    
    
}

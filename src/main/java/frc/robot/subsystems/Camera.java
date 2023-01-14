package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {

    boolean m_LimelightHasValidTarget = false;


    private NetworkTable m_networkTable;
     
    public static double pipeline = 0;
    
public void camera(){



   

   

    
}




@Override
public void periodic() {

    m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    m_networkTable.getEntry("pipeline").setNumber(pipeline);
    m_networkTable.getEntry("camMode").setNumber(0);
    
    
if (tv < 1.0)
{
    m_LimelightHasValidTarget = false;
}else{
    m_LimelightHasValidTarget = true;
}

   
    SmartDashboard.putBoolean("Is there a target: ", m_LimelightHasValidTarget);
    SmartDashboard.putNumber("Horizontal Offset: ", tx);
    SmartDashboard.putNumber("Verticle Offset: ", ty);
    SmartDashboard.putNumber("Target Area: ", ta);
    CameraServer.startAutomaticCapture();


}

public void setPipeline(int pipeline){
Camera.pipeline = pipeline;

}
    
}

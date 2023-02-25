package frc.robot.subsystems;

import java.util.stream.DoubleStream;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.limelight;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {

    boolean m_LimelightHasValidTarget = false;


    private NetworkTable m_networkTable;
     
    public double pipeline = 0;
    public double aprilTagPipeline;
    public String Alliance;

    private DriveSubsystem m_drive;

    DriverStation.Alliance alliance;
    double[] defaultDoubleArray = {0, 0, 0, 0, 0, 0};
    Pose2d defaultPose = new Pose2d(-5, -5, new Rotation2d());

    double[] robotPosX = new double[10];
    double[] robotPosY = new double[10];
    double[] robotPosYaw = new double[10];
    
    double[] tagPosX = new double[10];
    double[] tagPosY = new double[10];
    int[] tagIds = new int[10];
public void camera( DriveSubsystem m_drive){
    this.m_drive = m_drive;
    
   
    CameraServer.startAutomaticCapture();
   

    
}


public void CameraPipeline() {
    pipeline = 0;  
}

public void CameraToLimelight() {
    pipeline = 1;  
}

public void CameraToAprilTag() {
    pipeline = aprilTagPipeline;  
}

public double getFiducialID(){
    double tid= NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

    return tid;

}
public double[] getBotPose() {
    
    double[] botPose;
    
      
        botPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(defaultDoubleArray);

        if (botPose.length > 0) {
            return botPose;
        }
        else {
            return defaultDoubleArray;
        }
            

       
               }

public double getDetectionTimestamp() {
                
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("timestamp").getDouble(0);
                  
        }
            

        public Pose2d getRobotPose2d() {
            double[] pose = getBotPose();
            return new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
          }



          public Pose2d[] getRobotPoses2d() {
            Pose2d[] poseArray = {defaultPose};
        
           
              robotPosX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("Robot Pose X").getDoubleArray(new double[] {});
              robotPosY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("Robot Pose Y").getDoubleArray(new double[] {});
              robotPosYaw = NetworkTableInstance.getDefault().getTable("limelight").getEntry("Robot Pose Yaw").getDoubleArray(new double[] {});
              poseArray = new Pose2d[robotPosX.length];
              for (int i = 0; i < robotPosX.length; i++)
                poseArray[i] =
                    new Pose2d(robotPosX[i], robotPosY[i], Rotation2d.fromDegrees(robotPosYaw[i]));
                    return poseArray;
            }
        
            public int[] getTagIds() {
                var tags = tagIds;
               
                      double[] rawTags = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[] {});
                      tags = DoubleStream.of(rawTags).mapToInt(d -> (int) d).toArray();
                      return tags;
                  }
            
                  public double getValidTargetType() {
                      
                        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
                     
                    }

                    public double[] getCameraToTarget() {
                      
                        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
                     
                    }

                    public Pose3d getCameraPose3d() {
                        double[] pose = getCameraToTarget();
                        Pose2d pose1 =  new Pose2d(pose[0], pose[1], Rotation2d.fromDegrees(pose[5]));
                        return new Pose3d(pose1);
                        // Transform2d transform = new Transform2d()
                      }

                    public boolean getValidTarget() {
                        return getValidTargetType() > 0;
                      }
                
                  private void updateVisionPose() {
                    if (getValidTarget())
                      m_drive
                          .getOdometry()
                          .addVisionMeasurement(getRobotPose2d(), getDetectionTimestamp());
                  }   


                  public double getJSON() {
                    
                      
                        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getDouble(0);
                      
                    
                  }
              
          
        


@Override
public void periodic() {

    m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    m_networkTable.getEntry("pipeline").setNumber(pipeline);
    //m_networkTable.getEntry("camMode").setNumber(0);
    
    alliance = DriverStation.getAlliance();



    if(alliance == DriverStation.Alliance.Red){

        aprilTagPipeline = 2;
        Alliance = "Red";

    } else if(alliance == DriverStation.Alliance.Blue){

        aprilTagPipeline = 3;
        Alliance = "Blue";
    }
    
    
    
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
    SmartDashboard.putString("Alliance", Alliance);


updateVisionPose();

}




    
}

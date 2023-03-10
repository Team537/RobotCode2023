// package frc.robot.subsystems;


// import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
// import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

// import java.io.IOException;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.Vector;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.Constants.VisionConstants;


// public class PoseEstimatorSubsystem extends SubsystemBase {

//   // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
//   // you trust your various sensors. Smaller numbers will cause the filter to
//   // "trust" the estimate from that particular component more than the others. 
//   // This in turn means the particualr component will have a stronger influence
//   // on the final pose estimate.

//   /**
//    * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
//    * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
//    */
//   private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);

//   /**
//    * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
//    * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
//    */
//   private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 0.9);

//   private final DriveSubsystem m_drive;
//   private final SwerveDrivePoseEstimator poseEstimator;
//   private final Field2d field2d = new Field2d();
//   private final PhotonPoseEstimator photonPoseEstimator;

//   private double previousPipelineTimestamp = 0;
//   private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;
//   private boolean sawTag = false;

//   public PoseEstimatorSubsystem(PhotonCamera photonCamera, DriveSubsystem m_drive) {
//     this.m_drive = m_drive;
//     PhotonPoseEstimator photonPoseEstimator = null;
//     try {
//       var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
//       layout.setOrigin(originPosition);
//       if (photonCamera != null) {
//         photonPoseEstimator =
//             new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, photonCamera, VisionConstants.CAMERA_TO_ROBOT);
//       }
//     } catch(IOException e) {
//       DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
//       photonPoseEstimator = null;
//     }
//     this.photonPoseEstimator = photonPoseEstimator;

//     poseEstimator =  new SwerveDrivePoseEstimator(
//         SwerveConstants.kDriveKinematics,
//         m_drive.getHeadingRotation2d(),
//         m_drive.getModulePositions(),
//         new Pose2d(),
//         stateStdDevs,
//         visionMeasurementStdDevs);
//   }

//   public void addDashboardWidgets(ShuffleboardTab tab) {
//     tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
//     tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
//   }

//   /**
//    * Sets the alliance. This is used to configure the origin of the AprilTag map
//    * @param alliance alliance
//    */
//   public void setAlliance(Alliance alliance) {
//     boolean allianceChanged = false;
//     switch(alliance) {
//       case Blue:
//         allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
//         originPosition = OriginPosition.kBlueAllianceWallRightSide;
//         break;
//       case Red:
//         allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
//         originPosition = OriginPosition.kRedAllianceWallRightSide;
//         break;
//       default:
//         // No valid alliance data. Nothing we can do about it
//     }
//     if (photonPoseEstimator != null) {
//       photonPoseEstimator.getFieldTags().setOrigin(originPosition);
//     }
//     if (allianceChanged && sawTag) {
//       // The alliance changed, which changes the coordinate system.
//       // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
//       // needs to be transformed to the new coordinate system.
//       var newPose = flipAlliance(poseEstimator.getEstimatedPosition());
//       setCurrentPose(newPose);
//     }
//   }

//   @Override
//   public void periodic() {
//     // Update pose estimator with drivetrain sensors
//     poseEstimator.update(
//         m_drive.getHeadingRotation2d(),
//         m_drive.getModulePositions());
    
//     if (photonPoseEstimator != null) {
//       // Update pose estimator with the best visible target
//       photonPoseEstimator.update().ifPresent(estimatedRobotPose -> {
//         sawTag = true;
//         var estimatedPose = estimatedRobotPose.estimatedPose;
//         // Make sure we have a new measurement, and that it's on the field
//         if (estimatedRobotPose.timestampSeconds != previousPipelineTimestamp
//             && estimatedPose.getX() > 0.0 && estimatedPose.getX() <= VisionConstants.FIELD_LENGTH_METERS
//             && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= VisionConstants.FIELD_WIDTH_METERS) {
//           previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
//           poseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
//         }
//       });
//     }

//     Pose2d dashboardPose = getCurrentPose();
//     if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
//       // Flip the pose when red, since the dashboard field photo cannot be rotated
//       dashboardPose = flipAlliance(dashboardPose);
//     }
//     field2d.setRobotPose(dashboardPose);
//   }

//   private String getFomattedPose() {
//     var pose = getCurrentPose();
//     return String.format("(%.2f, %.2f) %.2f degrees", 
//         pose.getX(), 
//         pose.getY(),
//         pose.getRotation().getDegrees());
//   }

//   public Pose2d getCurrentPose() {
//     return poseEstimator.getEstimatedPosition();
//   }

//   /**
//    * Resets the current pose to the specified pose. This should ONLY be called
//    * when the robot's position on the field is known, like at the beginning of
//    * a match.
//    * @param newPose new pose
//    */
//   public void setCurrentPose(Pose2d newPose) {
//     poseEstimator.resetPosition(
//       m_drive.getHeadingRotation2d(),
//       m_drive.getModulePositions(),
//       newPose);
//   }

//   /**
//    * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
//    * what "forward" is for field oriented driving.
//    */
//   public void resetFieldPosition() {
//     setCurrentPose(new Pose2d());
//   }

//   /**
//    * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your
//    * alliance wall, so for 2023, the field elements are at different coordinates for each alliance.
//    * @param poseToFlip pose to transform to the other alliance
//    * @return pose relative to the other alliance's coordinate system
//    */
//   private Pose2d flipAlliance(Pose2d poseToFlip) {
//     return poseToFlip.relativeTo(new Pose2d(
//       new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
//       new Rotation2d(Math.PI)));
//   }

// }

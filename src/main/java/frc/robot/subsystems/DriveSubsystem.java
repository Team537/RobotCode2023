package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

import static frc.robot.Constants.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase {

    public static final double kMaxAngularSpeed = SwerveConstants.kMaxChassisRotationSpeed; // 3 meters per second

    private boolean isFieldOriented;
    private final double throttle = 0.8;
    private final double turningThrottle = 0.5;

    private int navXDebug = 0;

    private AHRS mNavX = new AHRS(SerialPort.Port.kMXP);

  

    PowerDistribution m_pdp;

    private double m_trajectoryTime;
    private Trajectory currentTrajectory;


   
    /**
     * Just like a graph's quadrants
     * 0 is Front Left
     * 1 is Front Right
     * 2 is Back Left
     * 3 is Back Right
     */
    private SwerveModule[] mSwerveModules = new SwerveModule[] {
        new SwerveModule(0, new TalonFX(SwerveConstants.frontLeftDrive), new TalonFX(SwerveConstants.frontLeftTurn), 0, true, false),
        new SwerveModule(1, new TalonFX(SwerveConstants.frontRightDrive), new TalonFX(SwerveConstants.frontRightTurn), 0, true, false), //true
        new SwerveModule(2, new TalonFX(SwerveConstants.backLeftDrive), new TalonFX(SwerveConstants.backLeftDrive), 0, true, false),
        new SwerveModule(3, new TalonFX(SwerveConstants.backRightDrive), new TalonFX(SwerveConstants.backRightTurn), 0, true, false) //true
    };
    SwerveModulePosition[] swerveModulePositions= new SwerveModulePosition[] { mSwerveModules[0].getPosition(),
      mSwerveModules[1].getPosition(), 
      mSwerveModules[2].getPosition(), 
      mSwerveModules[3].getPosition()};

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(SwerveConstants.kDriveKinematics, mNavX.getRotation2d(), swerveModulePositions
   );
    int navXSim = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    double simYaw = 0;

    
    

    public DriveSubsystem() {
      
     

    }


    public AHRS getNavX() {
        return mNavX;
    }

    public double getGyroRate() {
        return mNavX.getRate();
    }

    /**
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot.
     */
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDegrees());
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return mNavX.getRate();
    }

    /**
     * Returns the heading of the robot in degrees.
     *
     * @return the robot's heading in degrees, from 180 to 180
     */
    public double getHeadingDegrees() {
        try {
            return Math.IEEEremainder(-mNavX.getAngle(), 360);
        } catch (Exception e) {
            System.out.println("Cannot Get NavX Heading");
            return 0;
        }
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        for (int i = 0; i < 4; i++){
            mSwerveModules[i].resetEncoders();
        }

       
    }

    public void resetSimEncoders(){
      for (int i = 0; i < 4; i++){
        mSwerveModules[i].resetSimEncoders();
    }
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        mNavX.reset();
    }


    public SwerveModule getSwerveModule(int i) {
        return mSwerveModules[i];
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed *= SwerveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= SwerveConstants.kMaxSpeedMetersPerSecond;
        rot *= kMaxAngularSpeed;

        ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds( xSpeed, ySpeed, rot, getHeadingRotation2d()): new ChassisSpeeds(xSpeed, ySpeed, rot);

        
        SwerveModuleState[] swerveModuleStates = SwerveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeedMetersPerSecond);

        mSwerveModules[0].setDesiredState(swerveModuleStates[0]);
        mSwerveModules[1].setDesiredState(swerveModuleStates[1]);
        mSwerveModules[2].setDesiredState(swerveModuleStates[2]);
        mSwerveModules[3].setDesiredState(swerveModuleStates[3]);
    }

    public void setSwerveDriveNeutralMode(boolean mode) {
        for(int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setBrakeMode(mode);
        }
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, null, kFrontLeft, kEncoderDistancePerPulse, kEncoderCPR);
        mSwerveModules[0].setDesiredState(desiredStates[0]);
        mSwerveModules[1].setDesiredState(desiredStates[1]);
        mSwerveModules[2].setDesiredState(desiredStates[2]);
        mSwerveModules[3].setDesiredState(desiredStates[3]);
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {
        m_odometry.update(
            getHeadingRotation2d(), swerveModulePositions
            
        );

       
        // Update module positions based on the chassis' position, but keep the module heading
        for (int i = 0; i < mSwerveModules.length; i++) {
            var modulePositionFromChassis = SwerveConstants.modulePositions[i].rotateBy(getHeadingRotation2d()).plus(getPose().getTranslation());
            mSwerveModules[i].setPose(new Pose2d(modulePositionFromChassis, mSwerveModules[i].getHeading().plus(getHeadingRotation2d())));
        }


    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        m_odometry.resetPosition(rotation, new SwerveModulePosition[] {
          mSwerveModules[0].getPosition(),
          mSwerveModules[1].getPosition(), 
          mSwerveModules[2].getPosition(), 
          mSwerveModules[3].getPosition(),
          }, pose);

        for(int i = 0; i < mSwerveModules.length; i++) {
            mSwerveModules[i].setPose(pose);
            mSwerveModules[i].resetEncoders();
        }
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Chassis Angle",getHeadingDegrees());
        for(int i = 0; i < mSwerveModules.length; i++) {
            SmartDashboard.putNumber( "Swerve Module " + i + " Angle", mSwerveModules[i].getState().angle.getDegrees());
            SmartDashboard.putNumber( "Swerve Module " + i + " Speed", mSwerveModules[i].getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber( "X coordinate", getPose().getX());
        SmartDashboard.putNumber( "Y coordinate", getPose().getY());
    }

    @Override
    public void periodic() {
        sampleTrajectory();
        updateOdometry();
        updateSmartDashboard();
    }

    public Pose2d[] getModulePoses() {
        Pose2d[] modulePoses = {
            mSwerveModules[0].getPose(),
            mSwerveModules[1].getPose(),
            mSwerveModules[2].getPose(),
            mSwerveModules[3].getPose()
        };
        return modulePoses;
    }

    @Override
    public void simulationPeriodic() {
        SwerveModuleState[] moduleStates = {
            mSwerveModules[0].getState(),
            mSwerveModules[1].getState(),
            mSwerveModules[2].getState(),
            mSwerveModules[3].getState()
        };

        var chassisSpeed = SwerveConstants.kDriveKinematics.toChassisSpeeds(moduleStates);
        double chassisRotationSpeed = chassisSpeed.omegaRadiansPerSecond;

        simYaw += chassisRotationSpeed * 0.02;
        SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(navXSim, "Yaw"));
        angle.set(-Units.radiansToDegrees(simYaw));
    }

    private void sampleTrajectory() {
        if(DriverStation.isAutonomous()) {
            try {
                var currentTrajectoryState = currentTrajectory.sample(Timer.getFPGATimestamp() - startTime);

                System.out.println("Trajectory Time: " + (Timer.getFPGATimestamp() - startTime));
                System.out.println("Trajectory Pose: " + currentTrajectoryState.poseMeters);
                System.out.println("Trajectory Speed: " + currentTrajectoryState.velocityMetersPerSecond);
                System.out.println("Trajectory angular speed: " + currentTrajectoryState.curvatureRadPerMeter);
            } catch (Exception e) {

            }
        }

    }

    public void setTrajectoryTime(double trajectoryTime) {
        m_trajectoryTime = trajectoryTime;
    }

    double startTime;
    public void setCurrentTrajectory(Trajectory trajectory) {
        currentTrajectory = trajectory;
        startTime = Timer.getFPGATimestamp();
    }
}
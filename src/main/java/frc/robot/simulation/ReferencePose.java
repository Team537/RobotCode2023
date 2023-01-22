package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;

public class ReferencePose {
    static FieldSim m_fieldSim;

    public ReferencePose(FieldSim fieldSim) {
        m_fieldSim = fieldSim;
    }

    public static Pose2d getRobotFieldPose() {
        return m_fieldSim.getRobotPose();
    }
}
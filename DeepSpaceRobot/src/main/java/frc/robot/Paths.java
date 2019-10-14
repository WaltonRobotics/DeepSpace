package frc.robot;

import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Geometry.Translation2d;
import lib.Utils.Units;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;

public class Paths {

    public static Trajectory generateTestTrajectory(){
        final double startVelocity = 0;
        final double endVelocity = 0;
        final double maxVelocity = Units.feetToMeters(12);
        final double maxAccel = Units.feetToMeters(12);

        Pose2d startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Pose2d endPose = new Pose2d(4, 0, Rotation2d.fromDegrees(0));

        ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                waypoints,
                endPose,
                new ArrayList<>(),
                startVelocity,
                endVelocity,
                maxVelocity,
                maxAccel,
                false
        );
        return trajectory;
    }
}


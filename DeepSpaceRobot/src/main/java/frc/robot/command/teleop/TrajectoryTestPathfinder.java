package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import java.io.IOException;

import lib.Geometry.Pose2d;
import lib.Kinematics.ChassisSpeeds;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Utils.FollowFromFile;

import static frc.robot.Robot.drivetrain;

public class TrajectoryTestPathfinder extends Command {

    private DifferentialDriveKinematics kinematics;

    private Pose2d startingPose;
    private String trajectoryPath;

    private FollowFromFile pathFollower;

    public TrajectoryTestPathfinder(Pose2d startingPose, String trajectoryPath, double driveRadius) {
        requires(drivetrain);
        this.startingPose = startingPose;
        this.trajectoryPath = trajectoryPath;
        kinematics = new DifferentialDriveKinematics(driveRadius);
    }

    @Override
    protected void initialize() {

        Trajectory trajectory = null;

        try {
            trajectory = PathfinderFRC.getTrajectory(trajectoryPath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        double kBeta = 2.0;
        double kZeta = 0.7;

        this.pathFollower = new FollowFromFile(trajectory, kBeta, kZeta);
        drivetrain.getDriveOdometry().resetPosition(startingPose);
    }


    @Override
    protected void execute() {
        Pose2d currentPose = drivetrain.updateRobotPose();
        ChassisSpeeds chassisSpeeds = pathFollower.getRobotVelocity(currentPose);
        drivetrain.setVelocities(kinematics.toWheelSpeeds(chassisSpeeds).left, kinematics.toWheelSpeeds(chassisSpeeds).right);
    }

    @Override
    protected boolean isFinished() {
        return pathFollower.isFinished();
    }
}

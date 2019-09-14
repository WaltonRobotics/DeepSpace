package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystem.Drivetrain;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import java.io.IOException;
import java.util.stream.Stream;

import lib.Geometry.Pose2d;
import lib.Kinematics.ChassisSpeeds;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Kinematics.DifferentialDriveOdometry;
import lib.Utils.FollowFromFile;
import lib.Utils.VelocityPair;

import static frc.robot.Robot.drivetrain;


public class TrajectoryTestPathfinder extends Command {

    private DifferentialDriveOdometry odometry;

    private Pose2d startingPose;
    private String trajectoryPath;

    private FollowFromFile pathFollower;

    public TrajectoryTestPathfinder(Pose2d startingPose, String trajectoryPath) {
        requires(drivetrain);
        this.startingPose = startingPose;
        this.trajectoryPath = trajectoryPath;
        this.odometry = drivetrain.getDriveOdometry();
    }

    @Override
    protected void initialize() {

        Trajectory trajectory = null;

        try {
            trajectory = PathfinderFRC.getTrajectory(trajectoryPath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        double kBeta = 0.1;
        double kZeta = 2;

        this.pathFollower = new FollowFromFile(trajectory, kBeta, kZeta, 0.7);

    }


    @Override
    protected void execute() {

        Pose2d currentPose = drivetrain.updateRobotPose();
        ChassisSpeeds chassisSpeeds = pathFollower.getRobotVelocity(currentPose);

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}

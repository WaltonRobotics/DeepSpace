package frc.robot.command.teleop;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

import java.io.IOException;

import lib.Geometry.Pose2d;
import lib.Kinematics.ChassisSpeeds;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Utils.FollowFromFile;

import static frc.robot.Config.RamseteControllerConstants.K_BETA;
import static frc.robot.Config.RamseteControllerConstants.K_ZETA;
import static frc.robot.Robot.drivetrain;

public class FollowTrajectory extends Command {

    private DifferentialDriveKinematics kinematics;

    private Pose2d startingPose;
    private String trajectoryPath;

    private FollowFromFile pathFollower;

    public FollowTrajectory(Pose2d startingPose, String trajectoryPath, double driveRadius) {
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

        this.pathFollower = new FollowFromFile(trajectory, K_BETA, K_ZETA);
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

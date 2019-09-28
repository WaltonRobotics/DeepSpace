package lib.Utils;

import org.waltonrobotics.metadata.Pose;

import lib.Controller.RamseteController;
import lib.Geometry.Pose2d;
import lib.Kinematics.ChassisSpeeds;
import lib.trajectory.Trajectory;
import lib.trajectory.Trajectory.State;

public class TrajectoryFollower {

  private Trajectory trajectory;
  private double currentTime;
  private double dt;

  private RamseteController ramseteController;

  public TrajectoryFollower(Trajectory trajectory, double kBeta, double kZeta, double dt) {
    this.ramseteController = new RamseteController(kBeta, kZeta);
    this.trajectory = trajectory;
    this.currentTime = 0;
    this.dt = dt;
    ramseteController.setTolerance(new Pose2d());
  }

  /**
   * @param pose : The current pose of the robot
   * @return A velocity pair of the left and right wheel speeds
   */
  public ChassisSpeeds getRobotVelocity(Pose2d pose) {

    State desiredState = trajectory.sample(currentTime);
    State nextState = trajectory.sample(currentTime + dt);

    double trajX = desiredState.poseMeters.getTranslation().getX();
    double trajY = desiredState.poseMeters.getTranslation().getY();

    // Calculate X and Y error
    double xError = trajX - pose.getTranslation().getX();
    double yError = trajY - pose.getTranslation().getY();

    // Calculate Linear Velocity

    double sv = desiredState.velocityMetersPerSecond;

    // Calculate Angular Velocity

    double sw = isFinished() ? 0 : (nextState.poseMeters.getRotation().getDegrees() - desiredState.poseMeters.getRotation().getDegrees()) / dt;

    // Calculate linear and angular velocity based on errors

    ChassisSpeeds ramseteOutputs = ramseteController.calculate(pose, desiredState);

    currentTime += dt;

    return ramseteOutputs;
  }

  public boolean isFinished() {
    return ramseteController.atReference();
  }
}
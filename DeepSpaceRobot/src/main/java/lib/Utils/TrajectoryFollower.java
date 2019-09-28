package lib.Utils;

import lib.Controller.RamseteController;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
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
    ramseteController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(30)));
  }

  /**
   * @param pose : The current pose of the robot
   * @return A velocity pair of the left and right wheel speeds
   */
  public ChassisSpeeds getRobotVelocity(Pose2d pose) {

    State desiredState = trajectory.sample(currentTime);

    ChassisSpeeds ramseteOutputs = ramseteController.calculate(pose, desiredState);

    currentTime += dt;

    return ramseteOutputs;
  }

  public boolean isFinished() {
    return ramseteController.atReference();
  }
}
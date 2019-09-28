package lib.Utils;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import lib.Controller.RamseteController;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Kinematics.ChassisSpeeds;

public class FollowFromFile {

  private int currentSegIndex;
  private Segment currentSeg;
  private Trajectory trajectory;

  private RamseteController ramseteController;

  public FollowFromFile(Trajectory traj, double kBeta, double kZeta) {
      ramseteController = new RamseteController(kBeta, kZeta);

      trajectory = traj;
      currentSegIndex = 0;
      currentSeg = trajectory.segments[currentSegIndex];
      ramseteController.setTolerance(new Pose2d(0.03, 0.03, Rotation2d.fromDegrees(30)));
  }

  /**
   * @param pose : The current pose of the robot
   * @return A velocity pair of the left and right wheel speeds
   */
  public ChassisSpeeds getRobotVelocity(Pose2d pose) {

    // Update Segment Safely
    if (isFinished())
      return new ChassisSpeeds(0, 0, 0);

    // Calculate Linear Velocity of Segment

    double sv = currentSeg.velocity;

    // Calculate Angular Velocity of Segment

    double sw = (trajectory.segments[currentSegIndex + 1].heading - currentSeg.heading)/currentSeg.dt;

    // Calculate linear and angular velocity based on errors

    ChassisSpeeds ramseteOutputs = ramseteController
        .calculate(new Pose2d(currentSeg.x, currentSeg.y, new Rotation2d(currentSeg.heading)), pose, sv, sw);

    currentSegIndex++;

    return ramseteOutputs;
  }

  public boolean isFinished() {
    return currentSegIndex >= trajectory.length();
  }
}

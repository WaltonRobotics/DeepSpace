package frc.robot.command.auto;

import org.ghrobotics.lib.debug.LiveDashboard;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import lib.Controller.RamseteController;
import lib.Geometry.Pose2d;
import lib.Kinematics.ChassisSpeeds;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Kinematics.DifferentialDriveWheelSpeeds;
import lib.Utils.MotionPair;
import lib.Utils.Units;
import lib.trajectory.Trajectory;
import lib.trajectory.TrajectoryGenerator;

import static frc.robot.Config.RamseteControllerConstants.DRIVE_RADIUS;
import static frc.robot.Config.RamseteControllerConstants.K_BETA;
import static frc.robot.Config.RamseteControllerConstants.K_ZETA;
import static frc.robot.Config.RamseteControllerConstants.TOLERANCE_POSE;
import static frc.robot.Config.SmartMotionConstants.KT;
import static frc.robot.Robot.drivetrain;

public class FollowTrajectory extends Command {

  private DifferentialDriveKinematics kinematics;

  private Pose2d startingPose;
  private Trajectory trajectory;

  private double currentTime;
  private double dt;

  private RamseteController ramseteController;

  private Timer timer;

  private boolean backwards;

  private double leftVelocity;
  private double rightVelocity;
  private double leftAcceleration;
  private double rightAcceleration;

  private double previousLeftVelocity;
  private double previousRightVelocity;

  public FollowTrajectory(Trajectory trajectory) {
    requires(drivetrain);
    this.startingPose = trajectory.getStates().get(0).poseMeters;
    this.trajectory = trajectory;
    this.kinematics = new DifferentialDriveKinematics(DRIVE_RADIUS);
    this.ramseteController = new RamseteController(K_BETA, K_ZETA);
    this.currentTime = 0;
    this.dt = KT;
    this.leftVelocity = 0;
    this.rightVelocity = 0;
    this.leftAcceleration = 0;
    this.rightAcceleration = 0;
    this.previousLeftVelocity = 0;
    this.previousRightVelocity = 0;
    this.timer = new Timer();
  }

  @Override
  protected void initialize() {
    drivetrain.getDriveOdometry().resetPosition(startingPose);
    ramseteController.setTolerance(TOLERANCE_POSE);
    timer.start();
    LiveDashboard.INSTANCE.setFollowingPath(true);
  }

  @Override
  protected void execute() {
    Pose2d currentPose = drivetrain.updateRobotPose();
    MotionPair drivetrainMotions = getRobotMotions(currentPose);

    LiveDashboard.INSTANCE.setRobotX(Units.metersToFeet(currentPose.getTranslation().getX()));
    LiveDashboard.INSTANCE.setRobotY(Units.metersToFeet(currentPose.getTranslation().getY()));
    LiveDashboard.INSTANCE.setPathHeading(Units.metersToFeet(currentPose.getRotation().getRadians()));

    drivetrain.setVoltages(drivetrainMotions.getLeftVelocity(), drivetrainMotions.getLeftAcceleration(), drivetrainMotions.getRightVelocity(), drivetrainMotions.getRightAcceleration());
    //drivetrain.setSpeeds(getRobotSpeeds(currentPose));
  }

  @Override
  protected boolean isFinished() {
    return ramseteController.atReference();
  }

  @Override
  protected void end() {
    timer.stop();
    LiveDashboard.INSTANCE.setFollowingPath(false);
  }

  /**
   * @param currentPose : The current pose of the robot
   * @return A chassis speed of the robot
   */

  private MotionPair getRobotMotions(Pose2d currentPose) {

    currentTime = timer.get();

    Trajectory.State desiredState = trajectory.sample(currentTime);
    ChassisSpeeds ramseteOutputs = ramseteController.calculate(currentPose, desiredState);

    leftVelocity = kinematics.toWheelSpeeds(ramseteOutputs).leftMetersPerSecond;
    rightVelocity = kinematics.toWheelSpeeds(ramseteOutputs).rightMetersPerSecond;

    leftAcceleration = (leftVelocity - previousLeftVelocity) / dt;
    rightAcceleration = (rightVelocity - previousRightVelocity) / dt;

    previousLeftVelocity = leftVelocity;
    previousRightVelocity = rightVelocity;

    Pose2d referencePose = desiredState.poseMeters;

    LiveDashboard.INSTANCE.setPathX(Units.metersToFeet(referencePose.getTranslation().getX()));
    LiveDashboard.INSTANCE.setPathY(Units.metersToFeet(referencePose.getTranslation().getY()));
    LiveDashboard.INSTANCE.setPathHeading(referencePose.getRotation().getRadians());

    return new MotionPair(leftVelocity, leftAcceleration, rightVelocity, rightAcceleration);
  }

  private DifferentialDriveWheelSpeeds getRobotSpeeds(Pose2d currentPose) {
      currentTime = timer.get();

      Trajectory.State desiredState = trajectory.sample(currentTime);
      ChassisSpeeds ramseteOutputs = ramseteController.calculate(currentPose, desiredState);

      return new DifferentialDriveWheelSpeeds(kinematics.toWheelSpeeds(ramseteOutputs).leftMetersPerSecond, kinematics.toWheelSpeeds(ramseteOutputs).rightMetersPerSecond);
  }

  public double getLeftVelocity() {
    return leftVelocity;
  }

  public double getRightVelocity() {
    return rightVelocity;
  }

  public double getLeftAcceleration() {
    return leftAcceleration;
  }

  public double getRightAcceleration() {
    return rightAcceleration;
  }
}



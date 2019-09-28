package frc.robot.command.auto;

import edu.wpi.first.wpilibj.command.Command;
import lib.Controller.RamseteController;
import lib.Geometry.Pose2d;
import lib.Geometry.Rotation2d;
import lib.Kinematics.ChassisSpeeds;
import lib.Kinematics.DifferentialDriveKinematics;
import lib.Utils.MotionPair;
import lib.trajectory.Trajectory;

import static edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod;
import static frc.robot.Config.RamseteControllerConstants.K_BETA;
import static frc.robot.Config.RamseteControllerConstants.K_ZETA;
import static frc.robot.Config.SmartMotionConstants.KT;
import static frc.robot.Robot.drivetrain;

public class FollowTrajectory extends Command {

  private DifferentialDriveKinematics kinematics;

  private Pose2d startingPose;
  private Trajectory trajectory;

  private double currentTime;
  private double dt;

  private RamseteController ramseteController;

  private double leftVelocity;
  private double rightVelocity;
  private double leftAcceleration;
  private double rightAcceleration;

  private double previousLeftVelocity;
  private double previousRightVelocity;

  public FollowTrajectory(Trajectory trajectory, double driveRadius) {
    requires(drivetrain);
    this.startingPose = trajectory.getStates().get(0).poseMeters;
    this.trajectory = trajectory;
    this.kinematics = new DifferentialDriveKinematics(driveRadius);
    this.ramseteController = new RamseteController(K_BETA, K_ZETA);
    this.currentTime = 0;
    this.dt = KT;
    this.leftVelocity = 0;
    this.rightVelocity = 0;
    this.leftAcceleration = 0;
    this.rightAcceleration = 0;
    this.previousLeftVelocity = 0;
    this.previousRightVelocity = 0;
  }

  @Override
  protected void initialize() {
    drivetrain.getDriveOdometry().resetPosition(startingPose);
    ramseteController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(30)));
  }


  @Override
  protected void execute() {
    Pose2d currentPose = drivetrain.updateRobotPose();
    MotionPair drivetrainMotions = getRobotMotions(currentPose);
    drivetrain.setVoltages(drivetrainMotions.getLeftVelocity(), drivetrainMotions.getLeftAcceleration(), drivetrainMotions.getRightVelocity(), drivetrainMotions.getRightAcceleration());
  }

  @Override
  protected boolean isFinished() {
    return ramseteController.atReference();
  }

  /**
   * @param currentPose : The current pose of the robot
   * @return A chassis speed of the robot
   */

  private MotionPair getRobotMotions(Pose2d currentPose) {

    boolean atEnd = currentTime >= trajectory.getTotalTimeSeconds();

    Trajectory.State desiredState = trajectory.sample(currentTime);
    ChassisSpeeds ramseteOutputs = ramseteController.calculate(currentPose, desiredState);

    leftVelocity = kinematics.toWheelSpeeds(ramseteOutputs).leftMetersPerSecond;
    rightVelocity = kinematics.toWheelSpeeds(ramseteOutputs).rightMetersPerSecond;

    leftAcceleration = (leftVelocity - previousLeftVelocity) / dt;
    rightAcceleration = (rightVelocity - previousRightVelocity) / dt;

    previousLeftVelocity = leftVelocity;
    previousRightVelocity = rightVelocity;

    currentTime += dt;

    return new MotionPair(kinematics.toWheelSpeeds(ramseteOutputs).leftMetersPerSecond, leftAcceleration, rightVelocity, rightAcceleration);
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


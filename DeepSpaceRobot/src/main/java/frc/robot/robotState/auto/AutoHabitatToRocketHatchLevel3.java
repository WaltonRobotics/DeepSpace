package frc.robot.robotState.auto;

import static frc.robot.Config.Point.backup;
import static frc.robot.Config.Point.frontRocketR;
import static frc.robot.Config.SmartDashboardKeys.MOTION_BACKUP_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTION_BACKUP_X;
import static frc.robot.Config.SmartDashboardKeys.MOTION_BACKUP_Y;
import static frc.robot.Config.SmartDashboardKeys.MOTION_FRONT_ROCKET_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTION_FRONT_ROCKET_X;
import static frc.robot.Config.SmartDashboardKeys.MOTION_FRONT_ROCKET_Y;
import static frc.robot.Config.SmartDashboardKeys.MOTION_HATCH_PICKUP_ANGLE;
import static frc.robot.Config.SmartDashboardKeys.MOTION_HATCH_PICKUP_X;
import static frc.robot.Config.SmartDashboardKeys.MOTION_HATCH_PICKUP_Y;
import static frc.robot.Robot.currentRobot;
import static frc.robot.Robot.drivetrain;
import static frc.robot.Robot.godSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.robotState.HatchHandlingTransition;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ActiveState;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;
import org.waltonrobotics.command.SimpleSpline;
import org.waltonrobotics.metadata.Pose;

/**
 * @author Marius Juston
 **/
public class AutoHabitatToRocketHatchLevel3 extends AutonState {

  private double timeout;

  @Override
  public Pose getStartPose() {
    return new Pose(0, 0, StrictMath.toRadians(90));
  }

  @Override
  public Pose getEndPose() {
//    return frontRocketR;
    return new Pose(
        SmartDashboard.getNumber(MOTION_FRONT_ROCKET_X, frontRocketR.getX()),
        SmartDashboard.getNumber(MOTION_FRONT_ROCKET_Y, frontRocketR.getY()),
        Math.toRadians(
            SmartDashboard.getNumber(MOTION_FRONT_ROCKET_ANGLE, frontRocketR.getDegrees())
        ));
  }

  @Override
  public void setSubsystemLimits() {
    Robot.godSubsystem.setCurrentActiveState(ActiveState.HATCH_HANDLING);
    Robot.godSubsystem.getCargo().setCurrentTarget(CargoPosition.SAFE);
    Robot.godSubsystem.getHatch().setCurrentTarget(HatchPosition.DEPLOY);
    Robot.godSubsystem.getHatch().setLimits(HatchPosition.DEPLOY);

    Robot.godSubsystem.getElevator().setControlMode(ElevatorControlMode.AUTO);
    Robot.godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH_BASE);
    Robot.godSubsystem.getElevator().setLimits(ElevatorLevel.HATCH_BASE);

    Robot.godSubsystem.getHatch().setIntake(true);
  }

  @Override
  public void setMotionPath() {

    double distance = SmartDashboard.getNumber("Distance", 2);

    Pose offset = getStartPose().offset(distance);
    SimpleSpline goTwoRocket = SimpleSpline
        .pathFromPosesWithAngle(
            false, getStartPose(), offset, getEndPose());

    Pose backUpPosition = new Pose(
        SmartDashboard.getNumber(MOTION_BACKUP_X, backup.getX()),
        SmartDashboard.getNumber(MOTION_BACKUP_Y, backup.getY()),
        StrictMath.toRadians(
            SmartDashboard.getNumber(MOTION_BACKUP_ANGLE, backup.getDegrees())
        ));

    SimpleSpline goBackwards = SimpleSpline
        .pathFromPosesWithAngleAndScale(
            currentRobot.getMaxVelocity(),
            currentRobot.getMaxAcceleration(),
            true,
            .2,
            .2,
            getEndPose(), backUpPosition);

    Pose hatchPosition = new Pose(
        SmartDashboard.getNumber(MOTION_HATCH_PICKUP_X, backup.getX()),
        SmartDashboard.getNumber(MOTION_HATCH_PICKUP_Y, backup.getY()),
        StrictMath.toRadians(
            SmartDashboard.getNumber(MOTION_HATCH_PICKUP_ANGLE, backup.getDegrees())
        ));

    SimpleSpline goToHatch = SimpleSpline.pathFromPosesWithAngle(
        currentRobot.getMaxVelocity(),
        currentRobot.getMaxAcceleration(),
        false, backUpPosition, hatchPosition);

    SimpleSpline hatchToBackwards = SimpleSpline.pathFromPosesWithAngleAndScale(
        currentRobot.getMaxVelocity(),
        currentRobot.getMaxAcceleration(),
        true,
        1, 0.2,
        hatchPosition, backUpPosition);

    SimpleSpline backwardsToRocket = SimpleSpline.pathFromPosesWithAngleAndScale(
        currentRobot.getMaxVelocity(),
        currentRobot.getMaxAcceleration(),
        false, 0.2, 0.2,
        backUpPosition, getEndPose());

    getCommandGroup().addSequential(goTwoRocket);
    getCommandGroup().addSequential(goBackwards);
    getCommandGroup().addSequential(goToHatch);
    getCommandGroup().addSequential(hatchToBackwards);
    getCommandGroup().addSequential(backwardsToRocket);

    drivetrain.startControllerMotion(getStartPose());
    getCommandGroup().start();

    timeout = godSubsystem.getCurrentTime() + 2500;
  }

  @Override
  public State autonPeriodic() {
    if (getCommandGroup().isCompleted()) {
      return new HatchHandlingTransition();
    }

    if (godSubsystem.getCurrentTime() > timeout) {
//      Robot.godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH3);
    }

    return this;
  }

  @Override
  public void finish() {
    Robot.godSubsystem.getHatch().setIntake(false);
    getCommandGroup().cancel();
    drivetrain.clearControllerMotions();
    drivetrain.cancelControllerMotion();
  }
}

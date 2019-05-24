package frc.robot.robotState.auto;

import frc.robot.OI;
import frc.robot.robotState.HatchHandlingTransition;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.*;
import frc.robot.util.EnhancedBoolean;
import org.waltonrobotics.command.SimpleSpline;
import org.waltonrobotics.metadata.Pose;

import static frc.robot.Config.Point.backup;
import static frc.robot.Config.Point.frontRocketR;
import static frc.robot.Config.WaltonDashboardKeys.*;
import static frc.robot.Robot.*;

/**
 * @author Marius Juston
 **/
public class AutoHabitatToRocketHatch extends AutonState {

    private final EnhancedBoolean isGoBackwardsFinished = new EnhancedBoolean();
    private final EnhancedBoolean isGoToHatchFinished = new EnhancedBoolean();
    private final EnhancedBoolean isGoToRocketFinished = new EnhancedBoolean();
    private final EnhancedBoolean isHatchToBackwardsFinished = new EnhancedBoolean();
    private final EnhancedBoolean isBackwardsToRocketFinished = new EnhancedBoolean();
    private boolean hasRisen = false;
    private SimpleSpline goBackwards;
    private SimpleSpline goToHatch;
    private SimpleSpline goToRocket;
    private SimpleSpline hatchToBackwards;
    private SimpleSpline backwardsToRocket;
    private double timeout;
    private boolean hasRisen2;

    /**
     * Used for R -> L points. Easy and fast.
     *
     * @param p the point to negate its x
     * @return a new point with the x negated from p
     */
    private static Pose negateX(Pose p) {
        double angle = p.getAngle();
        // the new angle is the original angle but x is negated
        double newAngle = StrictMath.atan2(StrictMath.sin(angle), -StrictMath.cos(angle));
        if (newAngle < 0) {
            newAngle += 2.0 * Math.PI;
        }
        return new Pose(-p.getX(), p.getY(), newAngle);
    }

    private Pose getStartPose() {
        return new Pose(0, 0, StrictMath.toRadians(90.0));
    }

    private Pose getEndPose() {
//    return frontRocketR;
        return new Pose(
                waltonDashboard.getEntry(MOTION_FRONT_ROCKET_X).getNumber(frontRocketR.getX()).doubleValue(),
                waltonDashboard.getEntry(MOTION_FRONT_ROCKET_Y).getNumber(frontRocketR.getY()).doubleValue(),
                Math.toRadians(
                        waltonDashboard.getEntry(MOTION_FRONT_ROCKET_ANGLE).getNumber(frontRocketR.getDegrees()).doubleValue()
                ));
    }

    @Override
    public void setSubsystemLimits() {
        godSubsystem.getCargo().setControlMode(ClawControlMode.AUTO);
        godSubsystem.setCurrentActiveState(ActiveState.HATCH_HANDLING);
        godSubsystem.getCargo().setCurrentTarget(CargoPosition.SAFE);
        godSubsystem.getHatch().setControlMode(HatchControlMode.AUTO);
        godSubsystem.getHatch().setCurrentTarget(HatchPosition.DEPLOY);
        godSubsystem.getHatch().setLimits(HatchPosition.DEPLOY);

        godSubsystem.getElevator().setControlMode(ElevatorControlMode.AUTO);
        godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH_BASE);
        godSubsystem.getElevator().setLimits(ElevatorLevel.HATCH_BASE);

        godSubsystem.getHatch().setIntake(true);
    }

    @Override
    public void setMotionPath() {

        double distance = waltonDashboard.getEntry(MOTION_DISTANCE).getNumber(2.0).doubleValue();

        Pose startPose = getStartPose();
        Pose offset = getStartPose().offset(distance);
        Pose endPose = getEndPose();
        Pose backUpPosition = new Pose(
                waltonDashboard.getEntry(MOTION_BACKUP_X).getNumber(backup.getX()).doubleValue(),
                waltonDashboard.getEntry(MOTION_BACKUP_Y).getNumber(backup.getY()).doubleValue(),
                StrictMath.toRadians(
                        waltonDashboard.getEntry(MOTION_BACKUP_ANGLE).getNumber(backup.getDegrees()).doubleValue()
                ));
        Pose hatchPosition = new Pose(
                waltonDashboard.getEntry(MOTION_HATCH_PICKUP_X).getNumber(backup.getX()).doubleValue(),
                waltonDashboard.getEntry(MOTION_HATCH_PICKUP_Y).getNumber(backup.getY()).doubleValue(),
                StrictMath.toRadians(
                        waltonDashboard.getEntry(MOTION_HATCH_PICKUP_ANGLE).getNumber(backup.getDegrees()).doubleValue()
                ));
        Pose endPose2 = new Pose(2.52, 3.3, StrictMath.toRadians(60.0));

        if (!waltonDashboard.getEntry(IS_RIGHT_AUTON).getBoolean(true)) {
            startPose = negateX(startPose);
            offset = negateX(offset);
            endPose = negateX(endPose);
            backUpPosition = negateX(backUpPosition);
            hatchPosition = negateX(hatchPosition);
            endPose2 = negateX(endPose2);
        }

        goToRocket = SimpleSpline
                .pathFromPosesWithAngle(
                        false, startPose, offset, endPose);

        goBackwards = SimpleSpline
                .pathFromPosesWithAngleAndScale(
                        currentRobot.getMaxVelocity(),
                        currentRobot.getMaxAcceleration() / 2.0,
                        true,
                        0.2,
                        0.2,
                        endPose, backUpPosition);

        goToHatch = SimpleSpline.pathFromPosesWithAngle(
                currentRobot.getMaxVelocity(),
                currentRobot.getMaxAcceleration(),
                false, backUpPosition, hatchPosition);

        hatchToBackwards = SimpleSpline.pathFromPosesWithAngleAndScale(
                currentRobot.getMaxVelocity(),
                currentRobot.getMaxAcceleration(),
                true,
                1.0, 0.2,
                hatchPosition, backUpPosition);

        backwardsToRocket = SimpleSpline.pathFromPosesWithAngleAndScale(
                currentRobot.getMaxVelocity(),
                currentRobot.getMaxAcceleration() / 2.0,
                false, 0.2, 0.2,
                backUpPosition, endPose2);

        getCommandGroup().addSequential(goToRocket);
        getCommandGroup().addSequential(goBackwards);
        getCommandGroup().addSequential(goToHatch);
        getCommandGroup().addSequential(hatchToBackwards);
        getCommandGroup().addSequential(backwardsToRocket);

        SimpleSpline finishing = SimpleSpline
                .pathFromPosesWithAngleAndScale(
                        currentRobot.getMaxVelocity(),
                        currentRobot.getMaxAcceleration() / 2.0,
                        true,
                        0.2,
                        0.2,
                        endPose2, backUpPosition);
        getCommandGroup().addSequential(finishing);

        drivetrain.startControllerMotion(getStartPose());
        getCommandGroup().start();

        timeout = godSubsystem.getCurrentTime() + 2500L;
    }

    private void check() {
        isGoBackwardsFinished.set(goBackwards.getPath().isFinished());
        isGoToHatchFinished.set(goToHatch.getPath().isFinished());
        isGoToRocketFinished.set(goToRocket.getPath().isFinished());
        isHatchToBackwardsFinished.set(hatchToBackwards.getPath().isFinished());
        isBackwardsToRocketFinished.set(backwardsToRocket.getPath().isFinished());
    }

    @Override
    public State autonPeriodic() {
        if (getCommandGroup().isCompleted()) {
            return new HatchHandlingTransition();
        }

        check();

        if ((godSubsystem.getCurrentTime() >= timeout) && !hasRisen) {
            if (hasRisen2) {
                godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH2);
                hasRisen2 = false;
            } else {
                godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH3);
                hasRisen2 = true;
            }

            hasRisen = true;
        }

        if (isGoToRocketFinished.isRisingEdge()) {
            godSubsystem.getHatch().setIntake(false);
        } else if (isGoBackwardsFinished.isRisingEdge()) {
            godSubsystem.getElevator().setElevatorLevel(ElevatorLevel.HATCH_BASE);
        } else if (isGoToHatchFinished.isRisingEdge()) {
            godSubsystem.getHatch().setIntake(true);
            hasRisen = false;
            timeout = godSubsystem.getCurrentTime() + 2500L;
        } else if (isBackwardsToRocketFinished.isRisingEdge()) {
            godSubsystem.getHatch().setIntake(false);
        }

        boolean right = Math.abs(getRightYJoystick()) > 0.1;
        boolean left = Math.abs(getLeftYJoystick()) > 0.1;

        if (right || left) {
            return new HatchHandlingTransition();
        }

        return this;
    }

    private double getLeftYJoystick() {
        return (currentRobot.getLeftJoystickConfig().isInverted() ? -1 : 1) * OI.leftJoystick.getY();
    }

    private double getRightYJoystick() {
        return (currentRobot.getRightJoystickConfig().isInverted() ? -1 : 1) * OI.rightJoystick.getY();
    }

    @Override
    public void finish() {
//    Robot.godSubsystem.getHatch().setIntake(false);
        getCommandGroup().cancel();
        drivetrain.clearControllerMotions();
        drivetrain.cancelControllerMotion();
        godSubsystem.setAutonomousEnabled(false);
    }

    @Override
    public String toString() {
        return "AutoHabitatToRocketHatch{" +
                "isGoBackwardsFinished=" + isGoBackwardsFinished +
                ", isGoToHatchFinished=" + isGoToHatchFinished +
                ", isGoToRocketFinished=" + isGoToRocketFinished +
                ", isHatchToBackwardsFinished=" + isHatchToBackwardsFinished +
                ", isBackwardsToRocketFinished=" + isBackwardsToRocketFinished +
                ", hasRisen=" + hasRisen +
                ", goBackwards=" + goBackwards +
                ", goToHatch=" + goToHatch +
                ", goToRocket=" + goToRocket +
                ", hatchToBackwards=" + hatchToBackwards +
                ", backwardsToRocket=" + backwardsToRocket +
                ", timeout=" + timeout +
                ", hasRisen2=" + hasRisen2 +
                "} " + super.toString();
    }
}

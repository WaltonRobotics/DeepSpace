package frc.robot.command.teleop.driveMode;

import static frc.robot.Config.DriveSettings.DEADBAND;
import static frc.robot.OI.leftJoystick;
import static frc.robot.OI.rightJoystick;
import static frc.robot.Robot.responseFunctionChooser;

public abstract class DriveMode {

    public abstract void feed();

    /**
     * Note that by default the areas outside the deadband are not scaled from 0 to -1 / 1.
     */
    double applyDeadband(double value) {
        return Math.abs(value) > DEADBAND ? value : 0;
    }

    double applyResponseFunction(double value) {
        return responseFunctionChooser.getSelected().getOutput(value);
    }

    double getLeftJoystickY() {
        return -leftJoystick.getY();
    }

    double getRightJoystickY() {
        return -rightJoystick.getY();
    }

    /* The following methods are for drive modes with separated turn and throttle commands (i.e. Curvature/Arcade). */

    /**
     * The left joystick is used for throttle.
     */
    double getThrottle() {
        return getLeftJoystickY();
    }

    /**
     * The right joystick is used for turning.
     */
    double getTurn() {
        return rightJoystick.getX();
    }

}

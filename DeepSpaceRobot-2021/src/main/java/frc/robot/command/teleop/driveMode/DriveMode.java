package frc.robot.command.teleop.driveMode;

import edu.wpi.first.wpilibj.GenericHID;

import static frc.robot.OI.*;
import static frc.robot.Robot.driveInputDeviceChooser;

public abstract class DriveMode {

    public abstract void feed();

    /**
     * Note that by default the areas outside the deadband are not scaled from 0 to -1 / 1.
     */
    double applyDeadband(double value) {
        return Math.abs(value) > 0.05 ? value : 0;
    }

    double getLeftJoystickY() {
        if (driveInputDeviceChooser.getSelected().equals("Joysticks")) {
            return -leftJoystick.getY();
        }

        return -gamepad.getLeftY();
    }

    double getRightJoystickY() {
        if (driveInputDeviceChooser.getSelected().equals("Joysticks")) {
            return -rightJoystick.getY();
        }

        return -gamepad.getRawAxis(3);
    }

    /* The following methods are for drive modes with separated turn and throttle commands (i.e. Curvature/Arcade). */

    /**
     * The left joystick is used for throttle.
     */
    double getThrottle() {
        if (driveInputDeviceChooser.getSelected().equals("Joysticks")) {
            return -leftJoystick.getY();
        }

        return -gamepad.getLeftY();
    }

    /**
     * The right joystick is used for turning.
     */
    double getTurn() {
        if (driveInputDeviceChooser.getSelected().equals("Joysticks")) {
            return rightJoystick.getX();
        }

        return gamepad.getRightX();
    }

}

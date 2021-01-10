package frc.robot.command.teleop.driveMode;

import static frc.robot.Robot.drivetrain;

public class TankDrive extends DriveMode {

    @Override
    public void feed() {
        double leftOutput = applyResponseFunction(applyDeadband(getLeftJoystickY()));
        double rightOutput = applyResponseFunction(applyDeadband(getRightJoystickY()));

        drivetrain.setSpeeds(leftOutput, rightOutput);
    }

}

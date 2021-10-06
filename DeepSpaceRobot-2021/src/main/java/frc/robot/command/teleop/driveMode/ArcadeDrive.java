package frc.robot.command.teleop.driveMode;

import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Robot.drivetrain;

public class ArcadeDrive extends DriveMode {

    @Override
    public void feed() {
        double xSpeed = getThrottle();
        double zRotation = getTurn();

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        double leftCommand = MathUtil.clamp(leftMotorOutput, -1.0, 1.0);
        double rightCommand = MathUtil.clamp(rightMotorOutput, -1.0, 1.0);
        drivetrain.setSpeeds(leftCommand, rightCommand);
    }

}

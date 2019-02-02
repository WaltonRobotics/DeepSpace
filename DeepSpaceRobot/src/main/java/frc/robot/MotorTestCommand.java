package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2974.robot.subsystems.Drivetrain;

import static frc.robot.Robot.drivetrain;
import static frc.team2974.robot.RobotMap.motorLeft;
import static frc.team2974.robot.RobotMap.motorRight;

public class MotorTestCommand extends Command {

    private final int speed = 1;
    @Override
    protected void initialize() {
        drivetrain.setSpeeds(0, 0);
    }

    @Override
    protected void execute() {

        drivetrain.setSpeeds(speed, speed);

        assert(speed == motorLeft.get());
        assert(speed == motorRight.get());

    }

    @Override
    protected boolean isFinished() {
        drivetrain.reset();
        return true;
    }
}

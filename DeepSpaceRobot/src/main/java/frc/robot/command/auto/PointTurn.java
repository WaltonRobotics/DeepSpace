package frc.robot.command.auto;

import lib.Utils.PIDCommand;
import lib.Utils.PIDController;

import static frc.robot.Robot.drivetrain;

public class PointTurn extends PIDCommand {

    public PointTurn(double targetAngleDegrees) {
        super(new PIDController(0.8, 0, 0),
                drivetrain::getAngleDegrees,
                targetAngleDegrees,
                outPut -> drivetrain.setArcadeSpeeds(0 ,outPut)
             );

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(5, 2);
    }
}

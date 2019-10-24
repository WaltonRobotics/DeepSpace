package frc.robot.command.auto;

import edu.wpi.first.wpilibj.command.Command;
import lib.Utils.PIDCommand;
import lib.Utils.PIDController;

import static frc.robot.Robot.drivetrain;

public class PointTurn extends PIDCommand {

    private PIDController turnController;

    private double tolerance;
    private double setPoint;

    private double turnVelocity;

    public PointTurn(double targetAngleDegrees) {
        super(new PIDController(1, 0, 0),
                drivetrain::getAngleDegrees,
                targetAngleDegrees,
                outPut -> drivetrain.setArcadeSpeeds(0 ,outPut)
             );

        getController().enableContinuousInput(-180, 180);
        getController().setTolerance(5, 2);
    }

}

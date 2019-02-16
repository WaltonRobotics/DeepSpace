package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;

public class CompStartHatch implements State
{
    private double angle = Robot.godSubsystem.getHatch().getAngle();

    @Override
    public void initialize() {

    }

    @Override
    public State periodic() {

        if (HatchPosition.DEPLOY.isClose(angle)){
            Robot.godSubsystem.getHatch().setIntakePower(true);
        }
        else if(HatchPosition.DEPLOY.inRange(angle))
        {
            Robot.godSubsystem.getHatch().setIntakePower(false);
            return new HatchHandling();
        }

        return null;
    }

    @Override
    public void finish()
    {

    }
}

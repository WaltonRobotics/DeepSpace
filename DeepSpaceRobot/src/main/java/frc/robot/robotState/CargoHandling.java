package frc.robot.robotState;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

import static frc.robot.OI.*;


public class CargoHandling implements State {

    public boolean isFlipped() {
        return RobotMap.clawRotationMotor.getActiveTrajectoryPosition() > 90;
    }

    @Override
    public void initialize() {
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.CARGO_HANDLING);

    }

    @Override
    public State periodic() {

    if(intakeCargoButton.get()) {
        Robot.godSubsystem.intakeCargo();
    }

    else if(outtakeCargoButton.get()) {
        Robot.godSubsystem.outTakeCargo();
    }

    else if(flipCargoIntakeButton.get()) {

        if (isFlipped()) {
            Robot.godSubsystem.flipInClawSystem();
        } else {
            Robot.godSubsystem.flipOutClawSystem();
        }
    }

    return null;
}

    @Override
    public void finish() {
        Robot.godSubsystem.outTakeCargo();
    }
}

package frc.robot.robotState;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

import static frc.robot.OI.*;
import static frc.robot.RobotMap.*;

public class HatchHandling implements State {

    private double hatchUprightPosition = 100;
    private boolean looseState = false;
    private double looseStateDuration = 2;
    private double speedCap = 0.1;

    private boolean lastHatchButtonPressed;
    private boolean lastHatchIntakeButtonPressed;
    private boolean currentButtonPressed;

    Timer timer = new Timer();

    @Override
    public void initialize() {
        Robot.godSubsystem.setCurrentActiveState(ElevatorCargoHatchSubsystem.ActiveState.HATCH_HANDLING);
        Robot.godSubsystem.closeHatchIntake();
    }



    @Override
    public State periodic() {

        currentButtonPressed = hatchFlipOutButton.get();
        lastHatchIntakeButtonPressed = hatchIntakeButton.get();
        lastHatchButtonPressed = currentButtonPressed;

        if (!hatchSensor.get()) {
            looseState = true;
            while (looseState && timer.get() <= looseStateDuration) {
                Robot.drivetrain.setSpeeds(speedCap, speedCap);
                if (timer.get() == looseStateDuration) {
                    looseState = false;
                    Robot.drivetrain.setSpeeds(leftJoystick.getY(), rightJoystick.getY());
                }
            }
        }


        if (hatchFlipOutButton.get() && !lastHatchButtonPressed) {
            Robot.godSubsystem.flipInHatchIntake();
        } else if (hatchFlipOutButton.get() && lastHatchButtonPressed) {
            Robot.godSubsystem.flipOutHatchIntake();
        }

        if (hatchIntakeButton.get() && !lastHatchIntakeButtonPressed) {
            Robot.godSubsystem.openHatchIntake();
        } else if (hatchIntakeButton.get() && lastHatchIntakeButtonPressed) {
            Robot.godSubsystem.closeHatchIntake();
        }


        return null;
    }


    @Override
    public void finish() {
        Robot.godSubsystem.closeHatchIntake();
        Robot.godSubsystem.flipInHatchIntake();
    }
}

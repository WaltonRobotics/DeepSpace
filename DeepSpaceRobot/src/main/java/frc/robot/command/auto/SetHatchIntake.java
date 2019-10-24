package frc.robot.command.auto;

import edu.wpi.first.wpilibj.command.Command;

import static frc.robot.Robot.godSubsystem;

public class SetHatchIntake extends Command {

    private boolean state;
    private int ran;

    public SetHatchIntake(boolean closeOrOpen) {
        state = closeOrOpen;
        ran = 0;
    }

    @Override
    protected void initialize() {
        godSubsystem.getHatch().setIntake(state);
        ran++;

    }

    @Override
    protected boolean isFinished() {
        return ran >= 1;
    }
}

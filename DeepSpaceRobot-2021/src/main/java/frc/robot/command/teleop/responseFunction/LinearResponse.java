package frc.robot.command.teleop.responseFunction;

public class LinearResponse implements ResponseFunction {

    @Override
    public double getOutput(double input) {
        return input;
    }

}

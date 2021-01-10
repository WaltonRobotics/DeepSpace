package frc.robot.command.teleop.responseFunction;

@FunctionalInterface
public interface ResponseFunction {

    double getOutput(double input);

}

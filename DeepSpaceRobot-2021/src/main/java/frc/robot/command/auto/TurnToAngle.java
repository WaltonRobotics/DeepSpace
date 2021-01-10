package frc.robot.command.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

import static frc.robot.Robot.currentRobot;
import static frc.robot.Robot.drivetrain;

public class TurnToAngle extends Command {

    private double targetAngle;

    public TurnToAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        currentRobot.getTurnPIDController().reset(new TrapezoidProfile.State(0, 0));
        SmartDashboard.putNumber("Turn Setpoint", targetAngle);
    }

    @Override
    public void execute() {
        double turnRate = -currentRobot.getTurnPIDController().calculate(drivetrain.getAngle().getDegrees(), targetAngle);
        SmartDashboard.putNumber("Velocity error", currentRobot.getTurnPIDController().getVelocityError());
        SmartDashboard.putNumber("Position error", currentRobot.getTurnPIDController().getPositionError());
        System.out.println(currentRobot.getTurnPIDController().getPositionError());
        drivetrain.setArcadeSpeeds(0, turnRate);
    }

    @Override
    public void end() {
        drivetrain.setSpeeds(0, 0);
    }

    @Override
    public boolean isFinished() {
        return currentRobot.getTurnPIDController().atGoal();
    }
}

package frc.robot.robotState;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.state.State;
import frc.robot.subsystem.Drivetrain;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ClawControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorControlMode;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchControlMode;

public class Disabled implements State {
    @Override
    public void initialize() {
        Robot.godSubsystem.getHatch().setHatchControlMode(HatchControlMode.DISABLED);
        Robot.godSubsystem.getCargo().setClawControlMode(ClawControlMode.DISABLED);
        Robot.godSubsystem.getElevator().setElevatorControlMode(ElevatorControlMode.DISABLED);
    }

    @Override
    public State periodic() {
        if(Robot.godSubsystem.isEnabled()){
            return new TakeControl();
        }

        return this;
    }

    @Override
    public void finish() {
    }
}

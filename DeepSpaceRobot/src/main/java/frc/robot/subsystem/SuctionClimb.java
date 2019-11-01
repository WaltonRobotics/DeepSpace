package frc.robot.subsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import static frc.robot.RobotMap.*;

public class SuctionClimb extends Subsystem {

    private boolean sendIt;

    public SuctionClimb() {

    }

    public void engageKickStand() {
        climberSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void moveClimber(double speed) {
        climberElevatorMotor.set(speed);
    }

    public void startVacuum() {
        vacuumMotor.set(1);
    }

    @Override
    protected void initDefaultCommand() {

    }
}

package frc.robot.subsystem;


import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.robotState.Disabled;
import frc.robot.state.StateBuilder;
import frc.robot.util.Logger;

import static frc.robot.OI.elevatorDownButton;
import static frc.robot.OI.elevatorUpButton;
import static frc.robot.RobotMap.*;

public class ElevatorCargoHatchSubsystem extends Subsystem {

    private ActiveState currentActiveState = ActiveState.ROBOT_SWITCHED_ON;
    private Timer elevatorRuntime;

    private ElevatorControlMode elevatorControlMode;
    private Logger elevatorLogger;

    private StateBuilder stateMachine;

    // Inputs
    private boolean elevatorLastUpButtonPressed;
    private boolean elevatorLastDownButtonPressed;
    private boolean elevatorCurrentUpButtonPressed;
    private boolean elevatorCurrentDownButtonPressed;
    private int elevatorCurrentEncoderPosition;

    // Output
    private double elevatorCurrentPower;
    private double elevatorCurrentTarget;

    // ???

    public ElevatorCargoHatchSubsystem() {
        elevatorRuntime = new Timer();
        elevatorLogger = new Logger();
        elevatorControlMode = ElevatorControlMode.DISABLED;
        stateMachine = new StateBuilder(new Disabled());

        // Set sense of encoder
        // Set sense of motors
        // Set soft limits on
        // Configure elevator encoder

    }

    public ActiveState getCurrentActiveState() {
        return currentActiveState;
    }

    public void setCurrentActiveState(ActiveState currentActiveState) {
        this.currentActiveState = currentActiveState;
    }

    public void intakeCargo() {

        RobotMap.leftIntakeMotor.set(1);
        RobotMap.rightIntakeMotor.set(1);

    }

    public void outTakeCargo() {

        RobotMap.leftIntakeMotor.set(-1);
        RobotMap.rightIntakeMotor.set(-1);

    }

    public void flipOutClawSystem() {
        RobotMap.clawRotationMotor.set(ControlMode.MotionMagic, 1);
    }

    public void flipInClawSystem() {
        RobotMap.clawRotationMotor.set(ControlMode.MotionMagic, -1);
    }

    public void openHatchIntake() {
        if (!hatchIntake.get()) {
            hatchIntake.set(true);
        }
    }

    public void closeHatchIntake() {
        if (hatchIntake.get()) {
            hatchIntake.set(false);
        }
    }

    public void flipOutHatchIntake() {
        hatchRotationMotor.set(ControlMode.MotionMagic, 1);
    }

    public void flipInHatchIntake() {
        hatchRotationMotor.set(ControlMode.MotionMagic, -1);
    }

    public void resetElevator() {
        elevatorRuntime.reset();

        elevatorLastUpButtonPressed = false;
        elevatorLastDownButtonPressed = false;

        elevatorCurrentPower = 0.0;
        elevatorCurrentTarget = elevatorMotor.getSelectedSensorPosition(0);

        elevatorControlMode = ElevatorControlMode.AUTO;
    }

    public boolean isUpButtonPressed() {
        return elevatorCurrentUpButtonPressed;
    }

    public boolean isDownButtonPressed() {
        return elevatorCurrentDownButtonPressed;
    }

    public boolean wasUpButtonPressed() {
        return (elevatorCurrentUpButtonPressed != elevatorLastUpButtonPressed) && elevatorCurrentUpButtonPressed;
    }

    public boolean wasDownButtonPressed() {
        return (elevatorCurrentDownButtonPressed != elevatorLastDownButtonPressed) && elevatorCurrentDownButtonPressed;
    }

    /* Get raw height of elevator from encoder ticks. */
    public int getHeight() {
        return elevatorCurrentEncoderPosition;
    }

    public ElevatorLevel getLevel() {
        int currentHeight = getHeight();

        if (currentHeight >= ElevatorLevel.BASE.target && currentHeight < ElevatorLevel.LEVEL1.target)
            return ElevatorLevel.BASE;
        if (currentHeight >= ElevatorLevel.LEVEL1.target && currentHeight < ElevatorLevel.LEVEL2.target)
            return ElevatorLevel.LEVEL1;
        if (currentHeight >= ElevatorLevel.LEVEL2.target && currentHeight < ElevatorLevel.LEVEL3.target)
            return ElevatorLevel.LEVEL2;
        if (currentHeight >= ElevatorLevel.LEVEL3.target) return ElevatorLevel.LEVEL3;

        return ElevatorLevel.UNKNOWN;
    }

    public void setLevel(ElevatorLevel level) {
        elevatorCurrentTarget = level.getTarget();

        // Move this to set output
        elevatorMotor.set(ControlMode.Position, level.getTarget());
    }

    public void setPower(double percent) {
        elevatorCurrentPower = percent;
    }

    public ElevatorControlMode getControlMode() {
        return elevatorControlMode;
    }

    public void setControlMode(ElevatorControlMode controlMode) {
        this.elevatorControlMode = controlMode;
    }

    @Override
    public void periodic() {
        collectSensorData();
        processSensorData();
        output();
    }

    private void collectSensorData() {
        /* Read state of inputs. */
        elevatorLastUpButtonPressed = elevatorCurrentUpButtonPressed;
        elevatorCurrentUpButtonPressed = elevatorUpButton.get();

        elevatorLastDownButtonPressed = elevatorCurrentDownButtonPressed;
        elevatorCurrentDownButtonPressed = elevatorDownButton.get();

        elevatorCurrentEncoderPosition = elevatorMotor.getSelectedSensorPosition(0);
    }

    private void processSensorData() {
        stateMachine.step();
    }

    private void output() {
        // Here's where we actually set the motors etc...
        String logOutput = String.format("[%s]: Encoder height: %d, Current power: %f", elevatorRuntime.get(), getHeight(), elevatorCurrentPower);
        elevatorLogger.logInfo(logOutput);

        switch (elevatorControlMode) {
            case ZEROING:
                elevatorMotor.set(ControlMode.PercentOutput, elevatorCurrentPower);
                break;
            case AUTO:
                elevatorMotor.set(ControlMode.MotionMagic, elevatorCurrentTarget);
                break;
            case MANUAL:
                elevatorMotor.set(ControlMode.PercentOutput, elevatorCurrentPower);
                break;
            default:
                break;
        }
    }

    @Override
    protected void initDefaultCommand() {

    }

    public enum ElevatorLevel {
        UNKNOWN(0), BASE(100), LEVEL1(200), LEVEL2(300), LEVEL3(400);

        private double target;

        ElevatorLevel(double target) {
            this.target = target;
        }

        public double getTarget() {
            return target;
        }
    }

    public enum ElevatorControlMode {
        DISABLED, ZEROING, AUTO, MANUAL
    }

    public enum ActiveState {
        ROBOT_SWITCHED_ON,
        CARGO_HANDLING,
        DEFENSE,
        HATCH_HANDLING
    }

}

package frc.robot.config;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import java.util.HashMap;
import org.waltonrobotics.config.RobotConfig;
import org.waltonrobotics.config.TalonConfig;

public abstract class LimitedRobot extends RobotConfig {

  private final HashMap<Enum, LimitPair> limits = new HashMap<>(9);
  private final HashMap<Enum, Target> targets = new HashMap<>(21);

  protected LimitedRobot(String robotName) {
    super(robotName);
    initLimits();
    defineTargets();
  }

  public void addLimit(Enum anEnum, LimitPair limitPair) {
    limits.put(anEnum, limitPair);
  }

  public abstract BaseMotorControllerConfig getCargoSubsystemLimits();

  public abstract BaseMotorControllerConfig getHatchSubsystemLimits();

  public abstract BaseMotorControllerConfig getElevatorSubsystemLimits();

  public abstract TalonConfig getLeftIntakeMotorConfig();

  public abstract TalonConfig getRightIntakeMotorConfig();

  public abstract void initLimits();

  public void setCargoLimit(IMotorController talonSRX, Enum cargoType) {
    getCargoSubsystemLimits().setLimits(talonSRX, limits.get(cargoType));
  }

  public void setHatchLimit(IMotorController talonSRX, Enum cargoType) {
    getHatchSubsystemLimits().setLimits(talonSRX, limits.get(cargoType));
  }

  public void setElevatorLimit(IMotorController talonSRX, Enum cargoType) {
    getElevatorSubsystemLimits().setLimits(talonSRX, limits.get(cargoType));
  }

  public abstract TalonConfig getClimberMotorConfig();

  public abstract void defineTargets();

  public void addTarget(Enum anEnum, Target target) {
    targets.put(anEnum, target);
  }

  public Target getTarget(Enum target) {
    return targets.get(target);
  }

  public void setupController(BaseMotorController motorController, BaseMotorControllerConfig talonSRXConfig,
      Enum limitType) {
    motorController.setNeutralMode(talonSRXConfig.getNeutralMode());
    motorController.configSelectedFeedbackSensor(talonSRXConfig.getFeedbackSensor(), talonSRXConfig.getPIDIdx(),
        talonSRXConfig.getTimeout());
    motorController.setSensorPhase(talonSRXConfig.getSensorPhase()); // true for comp bot; false for practice
    motorController.setInverted(talonSRXConfig.isInverted());

    motorController.setStatusFramePeriod(talonSRXConfig.getStatusFramePeriod().value, 10, talonSRXConfig.getTimeout());
//    talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TIMEOUT);

    motorController.configNominalOutputForward(talonSRXConfig.getNominalOutputForward(), talonSRXConfig.getTimeout());
    motorController.configNominalOutputReverse(talonSRXConfig.getNominalOutputReverse(), talonSRXConfig.getTimeout());
    motorController.configPeakOutputForward(talonSRXConfig.getPeakOutputForward(), talonSRXConfig.getTimeout());
    motorController.configPeakOutputReverse(talonSRXConfig.getPeakOutputReverse(), talonSRXConfig.getTimeout());

    motorController.selectProfileSlot(talonSRXConfig.getProfileSlot(), talonSRXConfig.getPIDIdx());

    motorController.config_kP(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKP(), talonSRXConfig.getTimeout());
    motorController.config_kI(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKI(), talonSRXConfig.getTimeout());
    motorController.config_kD(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKD(), talonSRXConfig.getTimeout());
    motorController.config_kF(talonSRXConfig.getProfileSlot(), talonSRXConfig.getKF(), talonSRXConfig.getTimeout());
    motorController.configMotionCruiseVelocity(talonSRXConfig.getMotionCruiseVelocity(), talonSRXConfig.getTimeout());
    motorController.configMotionAcceleration(talonSRXConfig.getMotionAcceleration(), talonSRXConfig.getTimeout());
    motorController
        .configClosedLoopPeakOutput(talonSRXConfig.getProfileSlot(), talonSRXConfig.getClosedLoopPeakOutput());

    if (limitType != null) {
      LimitPair limitPair = limits.get(limitType);
      motorController
          .configForwardSoftLimitThreshold(limitPair.getForwardsSoftLimitThreshold(), talonSRXConfig.getTimeout());
      motorController
          .configReverseSoftLimitThreshold(limitPair.getReverseSoftLimitThreshold(), talonSRXConfig.getTimeout());
    }

    motorController.configForwardSoftLimitEnable(talonSRXConfig.isForwardsSoftLimitEnabled(), 10);
    motorController.configReverseSoftLimitEnable(talonSRXConfig.isReverseSoftLimitEnabled(), 10);

    /* pass false to FORCE OFF the feature.  Otherwise the enable flags above are honored */
    motorController.overrideLimitSwitchesEnable(talonSRXConfig.isOverrideLimitSwitchesEnabled());
  }

  @Override
  public String toString() {
    return "LimitedRobot{" +
        "limits=" + limits +
        ", targets=" + targets +
        "} " + super.toString();
  }
}

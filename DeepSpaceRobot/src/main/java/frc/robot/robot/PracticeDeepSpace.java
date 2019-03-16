package frc.robot.robot;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import frc.robot.Robot;
import frc.robot.config.BaseMotorControllerConfig;
import frc.robot.config.LimitPair;
import frc.robot.config.LimitedRobot;
import frc.robot.config.Target;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;
import org.waltonrobotics.config.Controls;
import org.waltonrobotics.config.EncoderConfig;
import org.waltonrobotics.config.TalonConfig;

public class PracticeDeepSpace extends LimitedRobot {

  public PracticeDeepSpace() {
    super("Practice DeepSpace");
  }

  @Override
  public EncoderConfig getRightEncoderConfig() {
    return new EncoderConfig() {
      @Override
      public double getDistancePerPulse() {
        return 0.000588608;
      }

      @Override
      public int getChannell1() {
        return 0;
      }

      @Override
      public int getChannell2() {
        return 1;
      }

      @Override
      public boolean isInverted() {
        return true;
      }
    };
  }

  @Override
  public EncoderConfig getLeftEncoderConfig() {
    return new EncoderConfig() {
      @Override
      public double getDistancePerPulse() {
        return 0.000588608;
      }

      @Override
      public int getChannell1() {
        return 2;
      }

      @Override
      public int getChannell2() {
        return 3;
      }

      @Override
      public boolean isInverted() {
        return false;
      }
    };
  }

  @Override
  public TalonConfig getLeftTalonConfig() {
    return new TalonConfig() {
      @Override
      public int getChanell() {
        return 3;
      }

      @Override
      public boolean isInverted() {
        return true;
      }
    };
  }

  @Override
  public TalonConfig getRightTalonConfig() {
    return new TalonConfig() {
      @Override
      public int getChanell() {
        return 1;
      }

      @Override
      public boolean isInverted() {
        return false;
      }
    };
  }

  @Override
  public Controls getRightJoystickConfig() {
    return () -> true;
  }

  @Override
  public Controls getLeftJoystickConfig() {
    return () -> true;
  }

  @Override
  public double getMaxAcceleration() {
    return 0;
  }

  @Override
  public double getMaxVelocity() {
    return 0;
  }

  @Override
  public double getKV() {
    return 1.0;
  }

  @Override
  public double getKAcc() {
    return 0;
  }

  @Override
  public double getKK() {
    return 0;
  }

  @Override
  public double getKS() {
    return 2.0;
  }

  @Override
  public double getKAng() {
    return 1.0;
  }

  @Override
  public double getKL() {
    return 0;
  }


  @Override
  public double getRobotWidth() {
    return 0.78;
  }

  @Override
  public double getRobotLength() {
    return 0.83;
  }

  @Override
  public boolean isCurrentRobot() {
    return !Robot.isCompBot;
  }

  @Override
  public BaseMotorControllerConfig getCargoSubsystemLimits() {
    return new BaseMotorControllerConfig() {
      @Override
      public int getDeviceID() {
        return 7;
      }

      @Override
      public NeutralMode getNeutralMode() {
        return NeutralMode.Brake;
      }

      @Override
      public FeedbackDevice getFeedbackSensor() {
        return FeedbackDevice.Analog;
      }

      @Override
      public boolean getSensorPhase() {
        return true;
      }

      @Override
      public boolean isInverted() {
        return true;
      }

      @Override
      public StatusFrameEnhanced getStatusFramePeriod() {
        return StatusFrameEnhanced.Status_10_MotionMagic;
      }

      @Override
      public double getNominalOutputForward() {
        return 0;
      }

      @Override
      public double getNominalOutputReverse() {
        return 0;
      }

      @Override
      public double getPeakOutputForward() {
        return 1.0;
      }

      @Override
      public double getPeakOutputReverse() {
        return -1.0;
      }

      @Override
      public int getProfileSlot() {
        return 0;
      }

      @Override
      public double getKP() {
        return 15.0;
      }

      @Override
      public double getKI() {
        return 0;
      }

      @Override
      public double getKD() {
        return 0;
      }

      @Override
      public double getKF() {
        return 0;
      }

      @Override
      public int getTimeout() {
        return 100;
      }

      @Override
      public int getPIDIdx() {
        return 0;
      }

      @Override
      public int getMotionCruiseVelocity() {
        return 2500;
      }

      @Override
      public int getMotionAcceleration() {
        return 2000;
      }

      @Override
      public boolean isReverseSoftLimitEnabled() {
        return true;
      }

      @Override
      public boolean isForwardsSoftLimitEnabled() {
        return true;
      }

      @Override
      public boolean isOverrideLimitSwitchesEnabled() {
        return false;
      }

      @Override
      public double getClosedLoopPeakOutput() {
        return 0.8;
      }
    };
  }

  @Override
  public BaseMotorControllerConfig getHatchSubsystemLimits() {
    return new BaseMotorControllerConfig() {
      @Override
      public int getDeviceID() {
        return 6;
      }

      @Override
      public NeutralMode getNeutralMode() {
        return NeutralMode.Brake;
      }

      @Override
      public FeedbackDevice getFeedbackSensor() {
        return FeedbackDevice.Analog;
      }

      @Override
      public boolean getSensorPhase() {
        return true;
      }

      @Override
      public boolean isInverted() {
        return false;
      }

      @Override
      public StatusFrameEnhanced getStatusFramePeriod() {
        return StatusFrameEnhanced.Status_10_MotionMagic;
      }

      @Override
      public double getNominalOutputForward() {
        return 0;
      }

      @Override
      public double getNominalOutputReverse() {
        return 0;
      }

      @Override
      public double getPeakOutputForward() {
        return 1.0;
      }

      @Override
      public double getPeakOutputReverse() {
        return -1.0;
      }

      @Override
      public int getProfileSlot() {
        return 0;
      }

      @Override
      public double getKP() {
        return 50.0;
      }

      @Override
      public double getKI() {
        return 0;
      }

      @Override
      public double getKD() {
        return 0;
      }

      @Override
      public double getKF() {
        return 0;
      }

      @Override
      public int getTimeout() {
        return 100;
      }

      @Override
      public int getPIDIdx() {
        return 0;
      }

      @Override
      public int getMotionCruiseVelocity() {
        return 2500;
      }

      @Override
      public int getMotionAcceleration() {
        return 2000;
      }

      @Override
      public boolean isReverseSoftLimitEnabled() {
        return true;
      }

      @Override
      public boolean isForwardsSoftLimitEnabled() {
        return true;
      }

      @Override
      public boolean isOverrideLimitSwitchesEnabled() {
        return false;
      }
    };
  }

  @Override
  public BaseMotorControllerConfig getElevatorSubsystemLimits() {
    return new BaseMotorControllerConfig() {
      @Override
      public int getDeviceID() {
        return 5;
      }

      @Override
      public NeutralMode getNeutralMode() {
        return NeutralMode.Brake;
      }

      @Override
      public FeedbackDevice getFeedbackSensor() {
        return FeedbackDevice.QuadEncoder;
      }

      @Override
      public boolean getSensorPhase() {
        return true;
      }

      @Override
      public boolean isInverted() {
        return false;
      }

      @Override
      public StatusFrameEnhanced getStatusFramePeriod() {
        return StatusFrameEnhanced.Status_10_MotionMagic;
      }

      @Override
      public double getNominalOutputForward() {
        return 0;
      }

      @Override
      public double getNominalOutputReverse() {
        return 0;
      }

      @Override
      public double getPeakOutputForward() {
        return 1.0;
      }

      @Override
      public double getPeakOutputReverse() {
        return -1.0;
      }

      @Override
      public int getProfileSlot() {
        return 0;
      }

      @Override
      public double getKP() {
        return 0.5;
      }

      @Override
      public double getKI() {
        return 0.00025;
      }

      @Override
      public double getKD() {
        return 0;
      }

      @Override
      public double getKF() {
        return 0.876857143;
      }

      @Override
      public int getTimeout() {
        return 100;
      }

      @Override
      public int getPIDIdx() {
        return 0;
      }

      @Override
      public int getMotionCruiseVelocity() {
        return 1600;
      }

      @Override
      public int getMotionAcceleration() {
        return 8000;
      }

      @Override
      public boolean isReverseSoftLimitEnabled() {
        return true;
      }

      @Override
      public boolean isForwardsSoftLimitEnabled() {
        return true;
      }

      @Override
      public boolean isOverrideLimitSwitchesEnabled() {
        return false;
      }
    };
  }

  @Override
  public TalonConfig getLeftIntakeMotorConfig() {
    return new TalonConfig() {
      @Override
      public int getChanell() {
        return 2;
      }

      @Override
      public boolean isInverted() {
        return false;
      }
    };
  }

  @Override
  public TalonConfig getRightIntakeMotorConfig() {
    return new TalonConfig() {
      @Override
      public int getChanell() {
        return 3;
      }

      @Override
      public boolean isInverted() {
        return true;
      }
    };
  }

  @Override
  public void initLimits() {
    this.addLimit(HatchPosition.CARGO_START, new LimitPair(-300, -568));
    this.addLimit(HatchPosition.HATCH_START, new LimitPair(-466, -568));
    this.addLimit(HatchPosition.SAFE, new LimitPair(-528, -578));
    this.addLimit(HatchPosition.DEPLOY, new LimitPair(-528, -772));

    this.addLimit(CargoPosition.SAFE, new LimitPair(655, 633));
    this.addLimit(CargoPosition.DEPLOY, new LimitPair(655/* 548*/, 380));
    this.addLimit(CargoPosition.CLIMB, new LimitPair(655, 349));

    this.addLimit(ElevatorLevel.CARGO_BASE, new LimitPair(31418, 1230));
    this.addLimit(ElevatorLevel.HATCH_BASE, new LimitPair(27500, 0));
  }

  @Override
  public TalonConfig getClimberMotorConfig() {
    return new TalonConfig() {
      @Override
      public int getChanell() {
        return 5;
      }

      @Override
      public boolean isInverted() {
        return false;
      }
    };
  }

  @Override
  public void defineTargets() {
    this.addTarget(HatchPosition.DEPLOY, new Target(-752, -650));
    this.addTarget(HatchPosition.SAFE, new Target(-548, -517));
    this.addTarget(HatchPosition.DEFENSE, new Target(-548, -371));
    this.addTarget(HatchPosition.HATCH_START, new Target(-486, -403));
    this.addTarget(HatchPosition.CARGO_START, new Target(-320, -300));

    int cargoOffset = 635;
    this.addTarget(CargoPosition.DEPLOY, new Target(-187 + cargoOffset, -85 + cargoOffset));
    this.addTarget(CargoPosition.CARGO_1, new Target(-144 + cargoOffset, 20 + cargoOffset));
    this.addTarget(CargoPosition.CARGO_2, new Target(-124 + cargoOffset, 20 + cargoOffset));
    this.addTarget(CargoPosition.CARGO_3, new Target(-95 + cargoOffset, 20 + cargoOffset));
    this.addTarget(CargoPosition.SAFE, new Target(cargoOffset, 20 + cargoOffset));
    this.addTarget(CargoPosition.CLIMB, new Target(-271 + cargoOffset, -85 + cargoOffset));

    this.addTarget(ElevatorLevel.CARGO_BASE, new Target(1784));

    this.addTarget(ElevatorLevel.CARGO_ROCKET, new Target(7308));
    this.addTarget(ElevatorLevel.CARGO_HAB, new Target(15200));
    this.addTarget(ElevatorLevel.CARGO2, new Target(18674));
    this.addTarget(ElevatorLevel.CARGO3, new Target(29404));
    this.addTarget(ElevatorLevel.CLIMB, new Target(8403));

    this.addTarget(ElevatorLevel.HATCH_BASE, new Target(1619));
    this.addTarget(ElevatorLevel.HATCH2, new Target(14465));
    this.addTarget(ElevatorLevel.HATCH3, new Target(27147));


  }
}

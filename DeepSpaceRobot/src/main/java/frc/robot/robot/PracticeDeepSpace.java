package frc.robot.robot;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import frc.robot.config.BaseMotorControllerConfig;
import frc.robot.config.LimitPair;
import frc.robot.config.LimitedRobot;
import frc.robot.config.Target;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;
import org.waltonrobotics.util.Controls;
import org.waltonrobotics.util.EncoderConfig;
import org.waltonrobotics.util.TalonConfig;

public class PracticeDeepSpace extends LimitedRobot {

  public PracticeDeepSpace() {
    super("Practice DeepSpace");
  }

  @Override
  public EncoderConfig getRightEncoderConfig() {
    return new EncoderConfig() {
      @Override
      public double getDistancePerPulse() {
        return 0.0;
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
        return false;
      }
    };
  }

  @Override
  public EncoderConfig getLeftEncoderConfig() {
    return new EncoderConfig() {
      @Override
      public double getDistancePerPulse() {
        return 0;
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
        return false;
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
        return true;
      }
    };
  }

  @Override
  public Controls getRightJoystickConfig() {
    return () -> false;
  }

  @Override
  public Controls getLeftJoystickConfig() {
    return () -> false;
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
    return 1;
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
    return 0;
  }

  @Override
  public double getKAng() {
    return 0;
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
    return true;
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
        return .3;
      }

      @Override
      public double getPeakOutputReverse() {
        return -.3;
      }

      @Override
      public int getProfileSlot() {
        return 0;
      }

      @Override
      public double getKP() {
        return 10;
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
        return 0.4;
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
        return 1;
      }

      @Override
      public double getPeakOutputReverse() {
        return -1;
      }

      @Override
      public int getProfileSlot() {
        return 0;
      }

      @Override
      public double getKP() {
        return 50;
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
        return 1;
      }

      @Override
      public double getPeakOutputReverse() {
        return -1;
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
        return 0;
      }

      @Override
      public double getKD() {
        return 0;
      }

      @Override
      public double getKF() {
        return 1.36;
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
        return 700;
      }

      @Override
      public int getMotionAcceleration() {
        return 1000;
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

    this.addLimit(ElevatorLevel.CARGO_BASE, new LimitPair(28750, 4615));
    this.addLimit(ElevatorLevel.HATCH_BASE, new LimitPair(26500, 0));
  }

  @Override
  public void defineTargets() {
    this.addTarget(HatchPosition.DEPLOY, new Target(-752, -650));
    this.addTarget(HatchPosition.SAFE, new Target(-548, -517));
    this.addTarget(HatchPosition.HATCH_START, new Target(-486, -403));
    this.addTarget(HatchPosition.CARGO_START, new Target(-320, -300));

    this.addTarget(CargoPosition.SAFE, new Target(644, 655));
    this.addTarget(CargoPosition.DEPLOY, new Target(471, 550));

    this.addTarget(ElevatorLevel.CARGO_BASE, new Target(4615));

    this.addTarget(ElevatorLevel.CARGO_ROCKET, new Target(9000));
    this.addTarget(ElevatorLevel.CARGO_HAB, new Target(11000));
    this.addTarget(ElevatorLevel.CARGO2, new Target(18000));
    this.addTarget(ElevatorLevel.CARGO3, new Target(27000));

    this.addTarget(ElevatorLevel.HATCH_BASE, new Target(0));
    this.addTarget(ElevatorLevel.HATCH2, new Target(13000));
    this.addTarget(ElevatorLevel.HATCH3, new Target(26000));


  }
}

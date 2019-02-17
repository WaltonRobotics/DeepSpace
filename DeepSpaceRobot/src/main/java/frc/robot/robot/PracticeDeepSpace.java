package frc.robot.robot;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import frc.robot.command.teleop.ElevatorCargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.Cargo;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;
import frc.robot.subsystem.TalonSRXConfig;
import frc.robot.subsystem.SubsystemTargets;
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
        return 0;
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
        return false;
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
  public TalonSRXConfig getCargoSubsystemLimits() {
    return new TalonSRXConfig() {
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
        return false;
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
        return 0;
      }

      @Override
      public double getPeakOutputReverse() {
        return 0;
      }

      @Override
      public int getProfileSlot() {
        return 0;
      }

      @Override
      public double getKP() {
        return 0;
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
        return 0;
      }

      @Override
      public int getMotionAcceleration() {
        return 0;
      }

      @Override
      public boolean isReverseSoftLimitEnabled() {
        return false;
      }

      @Override
      public boolean isForwardsSoftLimitEnabled() {
        return false;
      }

      @Override
      public boolean isOverrideLimitSwitchesEnabled() {
        return false;
      }
    };
  }

  @Override
  public TalonSRXConfig getHatchSubsystemLimits() {
    return new TalonSRXConfig() {
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
        return false;
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
        return 0;
      }

      @Override
      public double getPeakOutputReverse() {
        return 0;
      }

      @Override
      public int getProfileSlot() {
        return 0;
      }

      @Override
      public double getKP() {
        return 0;
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
        return 0;
      }

      @Override
      public int getMotionAcceleration() {
        return 0;
      }

      @Override
      public boolean isReverseSoftLimitEnabled() {
        return false;
      }

      @Override
      public boolean isForwardsSoftLimitEnabled() {
        return false;
      }

      @Override
      public boolean isOverrideLimitSwitchesEnabled() {
        return false;
      }
    };
  }

  @Override
  public TalonSRXConfig getElevatorSubsystemLimits() {
    return new TalonSRXConfig() {
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
        return 0;
      }

      @Override
      public double getPeakOutputReverse() {
        return 0;
      }

      @Override
      public int getProfileSlot() {
        return 0;
      }

      @Override
      public double getKP() {
        return 0;
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
        return 0;
      }

      @Override
      public int getMotionAcceleration() {
        return 0;
      }

      @Override
      public boolean isReverseSoftLimitEnabled() {
        return false;
      }

      @Override
      public boolean isForwardsSoftLimitEnabled() {
        return false;
      }

      @Override
      public boolean isOverrideLimitSwitchesEnabled() {
        return false;
      }
    };
  }

  @Override
  public SubsystemTargets getTargets() {
    return null;
  }

  @Override
  public void initLimits() {
    this.getLimits().put(HatchPosition.SAFE, new LimitPair(70, 90));
    this.getLimits().put(HatchPosition.DEPLOY, new LimitPair( 0, 90));
    this.getLimits().put(HatchPosition.CARGO_START, new LimitPair(90, 100));
    this.getLimits().put(HatchPosition.HATCH_START, new LimitPair(100, 110));
    this.getLimits().put(CargoPosition.SAFE, new LimitPair(70, 90));
    this.getLimits().put(CargoPosition.DEPLOY, new LimitPair(0, 90));
    this.getLimits().put(CargoPosition.ANGLED, new LimitPair(0, 10));
    this.getLimits().put(ElevatorLevel.BASE, new LimitPair(0, 100));
    this.getLimits().put(ElevatorLevel.CARGO1, new LimitPair(100, 200));
    this.getLimits().put(ElevatorLevel.CARGO2, new LimitPair(200, 300));
    this.getLimits().put(ElevatorLevel.CARGO3, new LimitPair(300, 400));
    this.getLimits().put(ElevatorLevel.HATCH1, new LimitPair(300, 400));
    this.getLimits().put(ElevatorLevel.HATCH2, new LimitPair(300, 400));
    this.getLimits().put(ElevatorLevel.HATCH3, new LimitPair(300, 400));

  }
}

package frc.robot.robot;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;
import frc.robot.subsystem.SusystemTargets;
import frc.robot.subsystem.TalonSRXConfig;
import org.waltonrobotics.util.Controls;
import org.waltonrobotics.util.EncoderConfig;
import org.waltonrobotics.util.TalonConfig;

public class CompDeepSpace extends LimitedRobot {

  public CompDeepSpace() {
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
        return 0;
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
        return 0;
      }

      @Override
      public int getChannell2() {
        return 0;
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
        return 0;
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
    return 0;
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
    return false;
  }

  @Override
  public TalonSRXConfig getCargoSubsystemLimits() {
    return new TalonSRXConfig() {
      @Override
      public NeutralMode getNeutralMode() {
        return null;
      }

      @Override
      public FeedbackDevice getFeedbackSensor() {
        return null;
      }

      @Override
      public boolean getSensorPhase() {
        return false;
      }

      @Override
      public boolean isInverted() {
        return false;
      }

      @Override
      public StatusFrameEnhanced getStatusFramePeriod() {
        return null;
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
        return 0;
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
    return null;
  }

  @Override
  public TalonSRXConfig getElevatorSubsystemLimits() {
    return null;
  }

  @Override
  public SusystemTargets getTargets() {
    return null;
  }

  @Override
  public void initLimits() {
  }
}

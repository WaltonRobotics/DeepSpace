package frc.robot.robot;

import frc.robot.config.BaseMotorControllerConfig;
import frc.robot.config.LimitedRobot;
import org.waltonrobotics.config.Controls;
import org.waltonrobotics.config.EncoderConfig;
import org.waltonrobotics.config.TalonConfig;

public class CompSteamWorks extends LimitedRobot {

  public CompSteamWorks() {
    super("SteamWorks Comp");
  }

  @Override
  public EncoderConfig getRightEncoderConfig() {
    return new EncoderConfig() {
      @Override
      public double getDistancePerPulse() {
        return 0.00055805;
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
        return true;
      }
    };
  }

  @Override
  public EncoderConfig getLeftEncoderConfig() {
    return new EncoderConfig() {
      @Override
      public double getDistancePerPulse() {
        return 0.00055805;
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
  public TalonConfig getLeftTalonConfig() {
    return new TalonConfig() {
      @Override
      public int getChanell() {
        return 0;
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
    return 4.0;
  }

  @Override
  public double getKV() {
    return 0.5349180715909608;
  }

  @Override
  public double getKAcc() {
    return 0.28150490814209445;
  }

  @Override
  public double getKK() {
    return -0.02306185102694297;
  }

  @Override
  public double getKS() {
    return 0.85;
  }

  @Override
  public double getKAng() {
    return 0.3;
  }

  @Override
  public double getKL() {
    return 2.0;
  }


  @Override
  public double getRobotWidth() {
    return 0.75;
  }

  @Override
  public double getRobotLength() {
    return 0.85;
  }

  @Override
  public boolean isCurrentRobot() {
    return false;
  }


  @Override
  public BaseMotorControllerConfig getCargoSubsystemLimits() {
    return null;
  }

  @Override
  public BaseMotorControllerConfig getHatchSubsystemLimits() {
    return null;
  }

  @Override
  public BaseMotorControllerConfig getElevatorSubsystemLimits() {
    return null;
  }

  @Override
  public TalonConfig getLeftIntakeMotorConfig() {
    return null;
  }

  @Override
  public TalonConfig getRightIntakeMotorConfig() {
    return null;
  }

  @Override
  public void initLimits() {

  }

  @Override
  public TalonConfig getClimberMotorConfig() {
    return null;
  }

  @Override
  public void defineTargets() {

  }
}

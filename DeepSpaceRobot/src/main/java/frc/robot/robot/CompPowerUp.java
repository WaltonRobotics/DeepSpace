package frc.robot.robot;


import frc.robot.subsystem.SubsystemLimits;
import org.waltonrobotics.util.Controls;
import org.waltonrobotics.util.EncoderConfig;
import org.waltonrobotics.util.RobotConfig;
import org.waltonrobotics.util.TalonConfig;

public class CompPowerUp extends LimitedRobot {

  public CompPowerUp() {
    super("Comp PowerUp");
  }

  @Override
  public EncoderConfig getRightEncoderConfig() {
    return new EncoderConfig() {
      @Override
      public double getDistancePerPulse() {
        return 0.0002045;
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
        return 0.0002045;
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
    return 4.5;
  }

  @Override
  public double getMaxVelocity() {
    return 4;
  }

  @Override
  public double getKV() {
    return 0.194350;
  }

  @Override
  public double getKAcc() {
    return 0.125;
  }

  @Override
  public double getKK() {
    return 0.194350;
  }

  @Override
  public double getKS() {
    return 2;
  }

  @Override
  public double getKAng() {
    return 1;
  }

  @Override
  public double getKL() {
    return 2;
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
  SubsystemLimits getSubsystemLimits() {
    return null;
  }
}

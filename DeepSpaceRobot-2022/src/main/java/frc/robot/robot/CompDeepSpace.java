package frc.robot.robot;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import frc.robot.config.BaseMotorControllerConfig;
import frc.robot.config.LimitPair;
import frc.robot.config.LimitedRobot;
import frc.robot.config.Target;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.CargoPosition;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.ElevatorLevel;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem.HatchPosition;
import frc.robot.lib.config.*;
import static frc.robot.Config.SmartDashboardKeys.*;

public class CompDeepSpace extends LimitedRobot {

    private final ProfiledPIDController turnPIDController = new ProfiledPIDController(0.8, 0, 0,
            new TrapezoidProfile.Constraints(360, 80));

    private final SimpleMotorFeedforward drivetrainFeedforward = new SimpleMotorFeedforward(0.178, 3.19, 0.462);

    private final PIDController leftVoltagePIDController = new PIDController(1.0, 0.0, 0.0);
    private final PIDController rightVoltagePIDController = new PIDController(1.0, 0.0, 0.0);

    private final PIDController leftVelocityPIDController = new PIDController(1.0, 0.0, 0.0);
    private final PIDController rightVelocityPIDController = new PIDController(1.0, 0.0, 0.0);

    public CompDeepSpace() {
        super("Comp DeepSpace");
    }

    @Override
    public EncoderConfig getRightEncoderConfig() {
        return new EncoderConfig() {
            @Override
            public double getDistancePerPulse() {
                return 0.000593397;
            }

            @Override
            public int getChannel1() {
                return 0;
            }

            @Override
            public int getChannel2() {
                return 1;
            }

            @Override
            public boolean isInverted() {
                return true;
            }
        };
    }

    public EncoderConfig getLeftEncoderConfig() {
        return new EncoderConfig() {
            @Override
            public double getDistancePerPulse() {
                return 0.000593397;
            }

            @Override
            public int getChannel1() {
                return 2;
            }

            @Override
            public int getChannel2() {
                return 3;
            }

            @Override
            public boolean isInverted() {
                return false;
            }
        };
    }

    public TalonConfig getLeftTalonConfig() {
        return new TalonConfig() {
            @Override
            public int getChannel() {
                return 3;
            }


            @Override
            public boolean isInverted() {
                return true;
            }
        };
    }

    public TalonConfig getRightTalonConfig() {
        return new TalonConfig() {
            @Override
            public int getChannel() {
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
        return SmartDashboard.getNumber(CONSTANTS_MAX_ACCELERATION, 3.0);
    }

    @Override
    public double getMaxVelocity() {
        return SmartDashboard.getNumber(CONSTANTS_MAX_VELOCITY, 1.5);
    }

    @Override
    public double getKV() {
        return SmartDashboard.getNumber(CONSTANTS_KV, 0.25);
    }

    @Override
    public double getKAcc() {
        return SmartDashboard.getNumber(CONSTANTS_KACC, 0.03);
    }

    @Override
    public double getKK() {
        return SmartDashboard.getNumber(CONSTANTS_KK, 0.01);
    }

    @Override
    public double getKS() {
        return SmartDashboard.getNumber(CONSTANTS_KS, 0.5);
    }

    @Override
    public double getKAng() {
        return SmartDashboard.getNumber(CONSTANTS_KANGLE, 1.05);
    }

    @Override
    public double getKL() {
        return SmartDashboard.getNumber(CONSTANTS_KL, 0.5);
    }

    @Override
    public boolean reverseAngleCalculation() {
        return false;
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
        return Robot.isCompBot;
    }

    @Override
    public double getKBeta() {
        return 0;
    }

    @Override
    public double getKZeta() {
        return 0;
    }

    @Override
    public double effectiveWheelbaseRadius() {
        return 0;
    }

    @Override
    public double wheelRadius() {
        return 0;
    }

    @Override
    public double robotMass() {
        return 0;
    }

    @Override
    public double robotMOI() {
        return 0;
    }

    @Override
    public double robotAngularDrag() {
        return 0;
    }

    @Override
    public MotorConfig leftMotorConfig() {
        return new MotorConfig() {
            @Override
            public int getChannel() {
                return 3;
            }

            @Override
            public boolean isInverted() {
                return true;
            }

            @Override
            public MotorParameters getMotorParameters() {
                return new MotorParameters(1, 0, 0);
            }
        };
    }

    @Override
    public MotorConfig rightMotorConfig() {
        return new MotorConfig() {
            @Override
            public int getChannel() {
                return 1;
            }

            @Override
            public boolean isInverted() {
                return false;
            }

            @Override
            public MotorParameters getMotorParameters() {
                return new MotorParameters(1, 0, 0);
            }
        };
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
                return 10.0;
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
                return 30.0;
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
                return 0;
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
            public int getChannel() {
                return 2;
            }

            @Override
            public boolean isInverted() {
                return true;
            }
        };
    }

    @Override
    public TalonConfig getRightIntakeMotorConfig() {
        return new TalonConfig() {
            @Override
            public int getChannel() {
                return 3;
            }


            @Override
            public boolean isInverted() {
                return false;
            }
        };
    }

    @Override
    public ProfiledPIDController getTurnPIDController() {
        return turnPIDController;
    }

    @Override
    public SimpleMotorFeedforward getDrivetrainFeedforward() {
        return drivetrainFeedforward;
    }

    @Override
    public PIDController getLeftVoltagePIDController() {
        return leftVoltagePIDController;
    }

    @Override
    public PIDController getRightVoltagePIDController() {
        return rightVoltagePIDController;
    }

    @Override
    public PIDController getLeftVelocityPIDController() {
        return leftVelocityPIDController;
    }

    @Override
    public PIDController getRightVelocityPIDController() {
        return rightVelocityPIDController;
    }

    @Override
    public void initLimits() {
//    this.addLimit(HatchPosition.CARGO_START, new LimitPair(-95, -412));
//    this.addLimit(HatchPosition.HATCH_START, new LimitPair(-275, -412));
//    this.addLimit(HatchPosition.SAFE, new LimitPair(-372, -412));
//    this.addLimit(HatchPosition.DEPLOY, new LimitPair(-372, -674));

        this.addLimit(HatchPosition.CARGO_START, new LimitPair(4000, -4000));
        this.addLimit(HatchPosition.HATCH_START, new LimitPair(4000, -4000));
        this.addLimit(HatchPosition.SAFE, new LimitPair(4000, -4000));
        this.addLimit(HatchPosition.DEPLOY, new LimitPair(4000, -4000));

        this.addLimit(CargoPosition.SAFE, new LimitPair(520, 500));
        this.addLimit(CargoPosition.DEPLOY, new LimitPair(520, 244));
        this.addLimit(CargoPosition.CLIMB, new LimitPair(520, 204));

        this.addLimit(ElevatorLevel.CARGO_BASE, new LimitPair(29857, 1230));
        this.addLimit(ElevatorLevel.HATCH_BASE, new LimitPair(26528, 0));
    }

    @Override
    public TalonConfig getClimberMotorConfig() {
        return new TalonConfig() {
            @Override
            public int getChannel() {
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
//    this.addTarget(HatchPosition.DEPLOY, new Target(-660, -551));
//    this.addTarget(HatchPosition.SAFE, new Target(-433, -371));
//    this.addTarget(HatchPosition.DEFENSE, new Target(-372, -371));
//    this.addTarget(HatchPosition.HATCH_START, new Target(-372, -233));
//    this.addTarget(HatchPosition.CARGO_START, new Target(-143, -113));
        this.addTarget(HatchPosition.DEPLOY, new Target(-609, -143));
        this.addTarget(HatchPosition.SAFE, new Target(-382, 37));
        this.addTarget(HatchPosition.DEFENSE, new Target(-310, 37));
        this.addTarget(HatchPosition.HATCH_START, new Target(-343, 175));
        this.addTarget(HatchPosition.CARGO_START, new Target(-343, 295));

        this.addTarget(CargoPosition.DEPLOY, new Target(278, 420));
        this.addTarget(CargoPosition.CARGO_1, new Target(353, 520));
        this.addTarget(CargoPosition.CARGO_2, new Target(371, 520));
        this.addTarget(CargoPosition.CARGO_3, new Target(390, 520));
        this.addTarget(CargoPosition.SAFE, new Target(511, 520));
        this.addTarget(CargoPosition.HAB, new Target(300, 520));
        this.addTarget(CargoPosition.CLIMB, new Target(204, 520));

        this.addTarget(ElevatorLevel.CARGO_BASE, new Target(2061));

        this.addTarget(ElevatorLevel.CARGO_ROCKET, new Target(7584));
        this.addTarget(ElevatorLevel.CARGO_HAB, new Target(15195));
        this.addTarget(ElevatorLevel.CARGO2, new Target(18448));
        this.addTarget(ElevatorLevel.CARGO3, new Target(29842));
        this.addTarget(ElevatorLevel.CLIMB, new Target(8403));

        this.addTarget(ElevatorLevel.HATCH_BASE, new Target(987)); // 1458
        this.addTarget(ElevatorLevel.HATCH2, new Target(13718));
        this.addTarget(ElevatorLevel.HATCH3, new Target(26528));


    }
}

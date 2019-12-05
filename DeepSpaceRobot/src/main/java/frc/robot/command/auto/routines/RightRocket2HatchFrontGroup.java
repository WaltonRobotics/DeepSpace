package frc.robot.command.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Paths;
import frc.robot.command.auto.SetElevatorLevel;
import frc.robot.command.auto.SetHatchIntake;
import frc.robot.command.auto.VisionAlign;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;
import lib.utils.RamseteCommand;
import lib.trajectory.Trajectory;

import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.drivetrain;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class RightRocket2HatchFrontGroup extends CommandGroup {
    public RightRocket2HatchFrontGroup() {
        Trajectory toFrontLv1 = Paths.RightRocketFront2.generateToFrontLv1();
        Trajectory backUpFrontLv1 = Paths.RightRocketFront2.generateBackUpFrontLv1();
        Trajectory toLoadingStationfromRocket1 = Paths.RightRocketFront2.generateToLoadingStation();
        Trajectory toFrontLv2FromLoading = Paths.RightRocketFront2.generateToFrontLv2FromLoading();
        Trajectory backUpFrontLv2 = Paths.RightRocketFront2.generateLevel2Backup();

        addSequential(new RamseteCommand(toFrontLv1,
                drivetrain::updateRobotPose,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new VisionAlign(1));
        addSequential(new SetHatchIntake(false));
        addSequential(new RamseteCommand(backUpFrontLv1,
                drivetrain::updateRobotPose,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new RamseteCommand(toLoadingStationfromRocket1,
                drivetrain::updateRobotPose,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new VisionAlign(0.9));
        addSequential(new SetHatchIntake(true));
        addSequential(new RamseteCommand(toFrontLv2FromLoading,
                drivetrain::updateRobotPose,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new SetElevatorLevel(ElevatorCargoHatchSubsystem.ElevatorLevel.HATCH2));
        addSequential(new VisionAlign(2.8));
        addSequential(new SetHatchIntake(false));
        addSequential(new RamseteCommand(backUpFrontLv2,
                drivetrain::updateRobotPose,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
    }
}

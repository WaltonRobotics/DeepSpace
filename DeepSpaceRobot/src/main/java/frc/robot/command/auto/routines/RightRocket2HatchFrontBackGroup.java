package frc.robot.command.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Paths.RightRocketBackAndFrontLv1;
import frc.robot.command.auto.SetHatchIntake;
import frc.robot.command.auto.VisionAlign;
import lib.Utils.RamseteCommand;
import lib.trajectory.Trajectory;

import static frc.robot.Config.SmartMotionConstants.*;
import static frc.robot.Robot.*;
import static frc.robot.RobotMap.encoderLeft;
import static frc.robot.RobotMap.encoderRight;

public class RightRocket2HatchFrontBackGroup extends CommandGroup {

    public RightRocket2HatchFrontBackGroup() {

        Trajectory toBackLv1 = RightRocketBackAndFrontLv1.generateToBackLv1();
        Trajectory backUpLv1 = RightRocketBackAndFrontLv1.generateBackUpFromBackLv1();
        Trajectory toLoadingStationfromLv1 = RightRocketBackAndFrontLv1.generateToLoadingStationFromBackLv1();
        Trajectory toLv1FromLoading = RightRocketBackAndFrontLv1.generateToFrontLv1FromLoading();

        addSequential(new RamseteCommand(toBackLv1,
                drivetrain::updateRobotPoseStartBackwards,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new VisionAlign(1.2));
        addSequential(new SetHatchIntake(false));
        addSequential(new RamseteCommand(backUpLv1,
                drivetrain::updateRobotPoseStartBackwards,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new RamseteCommand(toLoadingStationfromLv1,
                drivetrain::updateRobotPoseStartBackwards,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new VisionAlign(1.2));
        addSequential(new SetHatchIntake(true));
        addSequential(new RamseteCommand(toLv1FromLoading,
                drivetrain::updateRobotPoseStartBackwards,
                drivetrain.ramseteController,
                KS,
                KV,
                KA,
                drivetrain.differentialDriveKinematics,
                encoderLeft::getRate,
                encoderRight::getRate,
                drivetrain.m_leftPIDController,
                drivetrain.m_rightPIDController));
        addSequential(new VisionAlign(1.2));
        addSequential(new SetHatchIntake(false));
    }
}

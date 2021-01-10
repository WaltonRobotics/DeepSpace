package frc.robot.command.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Paths;
import frc.robot.command.auto.RamseteTrackingCommand;
import frc.robot.command.auto.VisionAlign;
import frc.robot.subsystem.ElevatorCargoHatchSubsystem;

import static frc.robot.Robot.godSubsystem;

public class RightRocket2HatchFrontGroup extends CommandGroup {
    public RightRocket2HatchFrontGroup() {
        Trajectory toFrontLv1 = Paths.RightRocketFront2.generateToFrontLv1();
        Trajectory backUpFrontLv1 = Paths.RightRocketFront2.generateBackUpFrontLv1();
        Trajectory toLoadingStationfromRocket1 = Paths.RightRocketFront2.generateToLoadingStation();
        Trajectory toFrontLv2FromLoading = Paths.RightRocketFront2.generateToFrontLv2FromLoading();
        Trajectory backUpFrontLv2 = Paths.RightRocketFront2.generateLevel2Backup();

        addSequential(new RamseteTrackingCommand(toFrontLv1, true));
        addSequential(new VisionAlign(1));
        addSequential(new InstantCommand(() -> godSubsystem.getHatch().setIntake(false)));
        addSequential(new RamseteTrackingCommand(backUpFrontLv1, true));
        addSequential(new RamseteTrackingCommand(toLoadingStationfromRocket1, true));
        addSequential(new VisionAlign(0.9));
        addSequential(new InstantCommand(() -> godSubsystem.getHatch().setIntake(true)));
        addSequential(new RamseteTrackingCommand(toFrontLv2FromLoading, true));
        addSequential(new InstantCommand(() -> godSubsystem.getElevator().setElevatorLevel(ElevatorCargoHatchSubsystem.ElevatorLevel.HATCH2)));
        addSequential(new VisionAlign(2.8));
        addSequential(new InstantCommand(() -> godSubsystem.getHatch().setIntake(false)));
        addSequential(new RamseteTrackingCommand(backUpFrontLv2, true));
    }
}

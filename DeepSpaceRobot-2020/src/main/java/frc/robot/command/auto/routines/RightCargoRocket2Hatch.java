package frc.robot.command.auto.routines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Paths;
import frc.robot.command.auto.RamseteTrackingCommand;
import frc.robot.command.auto.VisionAlign;

import static frc.robot.Robot.godSubsystem;

public class RightCargoRocket2Hatch extends CommandGroup {
    public RightCargoRocket2Hatch() {

        Trajectory toCargo1 = Paths.Right2HatchCargoRocket.generateToCargo1();
        Trajectory backUpCargo1 = Paths.Right2HatchCargoRocket.generateCargo1BackUp();
        Trajectory toLoadingStationfromCargo1 = Paths.Right2HatchCargoRocket.generateCargo1ToLoadingStation();
        Trajectory toLv1FromLoading = Paths.Right2HatchCargoRocket.generateToFrontLv1FromLoading();

        addSequential(new RamseteTrackingCommand(toCargo1, true));
        addSequential(new VisionAlign(1.5));
        addSequential(new InstantCommand(() -> godSubsystem.getHatch().setIntake(false)));
        addSequential(new RamseteTrackingCommand(backUpCargo1, true));
        addSequential(new RamseteTrackingCommand(toLoadingStationfromCargo1, true));
        addSequential(new VisionAlign(1.8));
        addSequential(new InstantCommand(() -> godSubsystem.getHatch().setIntake(true)));
        addSequential(new RamseteTrackingCommand(toLv1FromLoading, true));
        addSequential(new VisionAlign(3));
        addSequential(new InstantCommand(() -> godSubsystem.getHatch().setIntake(false)));
    }
}

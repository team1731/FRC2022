package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.Constants.InputRange;

public class LaunchRangeCommand extends CommandBase {
    private final LaunchSubsystem _launch;
    private final InputRange _input;

    public LaunchRangeCommand(LaunchSubsystem launch, InputRange input) {
        _launch = launch;
        _input = input;
        addRequirements(launch);
    }

    @Override
    public void initialize() {
        _launch.setLaunch(_input);
    }

    @Override
    public void end(boolean interrupted) {
        //_launch.stopLaunch();
    }
}

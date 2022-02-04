package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.Constants.InputRange;

public class LaunchRunCommand extends CommandBase {
    private final LaunchSubsystem _launch;

    public LaunchRunCommand(LaunchSubsystem launch) {
        _launch = launch;
        addRequirements(launch);
    }

    @Override
    public void initialize() {
        _launch.runLaunch(InputRange.HOME);
    }

    @Override
    public void end(boolean interrupted) {
    }
}

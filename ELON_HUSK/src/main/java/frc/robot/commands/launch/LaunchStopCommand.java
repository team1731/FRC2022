package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;

public class LaunchStopCommand extends CommandBase {
    private final LaunchSubsystem _launch;

    public LaunchStopCommand(LaunchSubsystem launch) {
        _launch = launch;
        addRequirements(launch);
    }

    @Override
    public void initialize() {
        _launch.stopLaunch();
    }

    @Override
    public void end(boolean interrupted) {
    }
}

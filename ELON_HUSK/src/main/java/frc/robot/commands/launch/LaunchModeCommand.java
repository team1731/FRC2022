package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LaunchSubsystem;

public class LaunchModeCommand extends InstantCommand {
    private final LaunchSubsystem _launch;

    public LaunchModeCommand(LaunchSubsystem launch) {
        _launch = launch;
        addRequirements(launch);
    }

    @Override
    public void initialize() {
        _launch.manualMode(true); // enable manual mode
    }

    @Override
    public void end(boolean interrupted) {
        _launch.manualMode(false); // disable manual mode
    }
}

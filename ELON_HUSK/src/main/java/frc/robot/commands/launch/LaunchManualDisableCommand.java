package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;

public class LaunchManualDisableCommand extends CommandBase {
    private final LaunchSubsystem _launch;

    public LaunchManualDisableCommand(LaunchSubsystem launch) {
        _launch = launch;
        addRequirements(launch);
    }

    @Override
    public void initialize() {
        _launch.manualMode(false); // disable manual mode
    }

    @Override
    public void end(boolean interrupted) {
    }
}

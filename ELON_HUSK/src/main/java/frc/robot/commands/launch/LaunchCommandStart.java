package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;

public class LaunchCommandStart extends CommandBase {
    private final LaunchSubsystem _launch;
    private double _stick;

    public LaunchCommandStart(LaunchSubsystem launch, double stick) {
        _launch = launch;
        _stick = stick;
    }

    @Override
    public void initialize() {
        _launch.runLaunch(_stick, 0.0);
    }

	@Override
	public void end(boolean interrupted) {
	}

    @Override
	public boolean isFinished() {
		return true;
	}
}
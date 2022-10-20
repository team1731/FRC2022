package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;

public class LaunchBallCommandStart extends CommandBase {
    private final LaunchSubsystem _launch;

    public LaunchBallCommandStart(LaunchSubsystem launch) {
        _launch = launch;
    }

    @Override
    public void initialize() {
        _launch.runLaunchBallAuto();
    }

	@Override
	public void end(boolean interrupted) {
	}

    @Override
	public boolean isFinished() {
		return true;
	}
}

    

package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;

public class LaunchCommandStart extends CommandBase {
    private final LaunchSubsystem _launch;
    private double _stick;
    private boolean _useVision;

    public LaunchCommandStart(LaunchSubsystem launch, double stick, boolean useVision) {
        _launch = launch;
        _stick = stick;
        _useVision = useVision;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        _launch.runLaunch(_stick, 0.0,_useVision);
    }

	@Override
	public void end(boolean interrupted) {
	}

    @Override
	public boolean isFinished() {
		return false;
	}
}
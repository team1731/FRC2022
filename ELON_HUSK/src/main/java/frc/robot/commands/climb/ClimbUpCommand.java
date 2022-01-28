package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbUpCommand extends CommandBase {

	protected final ClimbSubsystem _climb;

	public ClimbUpCommand(ClimbSubsystem climb) {
		_climb = climb;
		addRequirements(climb);
	}

	@Override
	public void initialize() {
		_climb.setInputDirection(ClimbSubsystem.InputDirection.UP);
	}

	@Override
    public void end(boolean interrupted) {
        _climb.setInputDirection(ClimbSubsystem.InputDirection.NEUTRAL);
    }

}

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDownCommand extends InstantCommand {

	private final ClimbSubsystem _climb;

	public ClimbDownCommand(ClimbSubsystem climb) {
		_climb = climb;
		addRequirements(climb);
	}

	@Override
	public void initialize() {
		_climb.setInputDirection(ClimbSubsystem.InputDirection.DOWN);
	}

}

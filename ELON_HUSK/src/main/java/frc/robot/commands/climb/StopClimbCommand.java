package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class StopClimbCommand extends InstantCommand {

	private final ClimbSubsystem _climb;

	public StopClimbCommand(ClimbSubsystem climb) {
		_climb = climb;
		addRequirements(climb);
	}

	@Override
	public void initialize() {
		_climb.setInputDirection(ClimbSubsystem.InputDirection.NEUTRAL);
	}

}

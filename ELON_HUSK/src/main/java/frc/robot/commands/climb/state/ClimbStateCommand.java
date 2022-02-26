package frc.robot.commands.climb.state;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public abstract class ClimbStateCommand extends CommandBase {

	protected final ClimbSubsystem _climb;

	public ClimbStateCommand(ClimbSubsystem climb){
		_climb = climb;
	}

	public abstract ClimbSubsystem.State getNextState();
	@Override
	public abstract void initialize();
	@Override
	public abstract boolean isFinished();

}

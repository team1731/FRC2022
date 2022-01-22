package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public abstract class ClimbCommand extends CommandBase {
    protected final ClimbSubsystem _climb;

    public ClimbCommand(ClimbSubsystem climb){
        _climb = climb;
        addRequirements(climb);
    }

    @Override
    public abstract void initialize();

    @Override
    public void end(boolean interrupted) {
        _climb.setInputState(ClimbSubsystem.InputState.NEUTRAL);
    }
}

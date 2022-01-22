package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbSubsystem;

public class ClimbUpCommand extends ClimbCommand {

    public ClimbUpCommand(ClimbSubsystem climb){
        super(climb);
    }

    @Override
    public void initialize() {
        _climb.setInputState(ClimbSubsystem.InputState.UP);
    }
}

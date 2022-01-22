package frc.robot.commands.climb;

import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDownCommand extends ClimbCommand {

    public ClimbDownCommand(ClimbSubsystem climb){
        super(climb);
    }

    @Override
    public void initialize() {
        _climb.setInputDirection(ClimbSubsystem.InputDirection.DOWN);
    }
}

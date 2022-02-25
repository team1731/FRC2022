package frc.robot.commands.climb.state;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.IRSensor;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.State;

public class ReadyStateCommand extends ClimbStateCommand {

    public ReadyStateCommand(ClimbSubsystem climb){
        super(climb);
    }

    @Override
    public State getNextState() {
        // TODO: Create other states
        return State.EXTEND;
    }

    @Override
    public void initialize() {
        // TODO: Need ClimbSubsystem methods
    }

    @Override
    public boolean isFinished() {
        // TODO: transition
        return false;
    }
    


}

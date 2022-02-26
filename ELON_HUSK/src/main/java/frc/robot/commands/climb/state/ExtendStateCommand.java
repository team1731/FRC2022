package frc.robot.commands.climb.state;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.IRSensor;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.State;

public class ExtendStateCommand extends ClimbStateCommand {

    private final IRSensor _sensor;
    private double _timer;

    public ExtendStateCommand(ClimbSubsystem climb, IRSensor sensor){
        super(climb);
        _sensor = sensor;
    }

    @Override
    public State getNextState() {
        // TODO: Create other states
        return null;
    }

    @Override
    public void initialize() {
        // TODO: Need ClimbSubsystem methods
        _timer = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(!_sensor.isTriggered()){
            _timer = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: transition
        // Use a timer to ensure the sensor has been triggered for a little bit before transitioning to next state
        return _climb.getSensorOveride() || (_sensor != null && _sensor.isTriggered() && _timer > 0.5);
    }
    


}

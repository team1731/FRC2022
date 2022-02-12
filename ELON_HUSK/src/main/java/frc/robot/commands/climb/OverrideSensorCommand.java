package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class OverrideSensorCommand extends CommandBase {
    
    private final ClimbSubsystem _climb;

    public OverrideSensorCommand(ClimbSubsystem climb){
        _climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        _climb.setSensorOverride(true);
    }
    
    @Override
    public void end(boolean interrupted) {
        _climb.setSensorOverride(false);
    }

}

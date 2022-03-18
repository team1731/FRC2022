package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RewindClimbComand extends CommandBase {
    private final ClimbSubsystem _climb;

    public RewindClimbComand(ClimbSubsystem climb) {
        _climb = climb;
    }

    @Override
    public void initialize() {
        _climb.startRewind();
    }

    @Override
    public void end(boolean interrupted) {
        _climb.stopRewind();
    }
}

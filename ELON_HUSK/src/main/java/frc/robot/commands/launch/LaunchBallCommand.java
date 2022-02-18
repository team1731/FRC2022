package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LaunchSubsystem;

public class LaunchBallCommand extends CommandBase {
    private final LaunchSubsystem _launch;

    public LaunchBallCommand(LaunchSubsystem launch) {
        _launch = launch;
        addRequirements(launch);
    }

    @Override
    public void initialize() {
        _launch.runLaunchBall();
    }

    @Override
    public void end(boolean interrupted) {
        _launch.stopLaunchBall();
    }
}

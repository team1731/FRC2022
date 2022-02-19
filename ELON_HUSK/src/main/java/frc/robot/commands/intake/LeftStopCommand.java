package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class LeftStopCommand extends CommandBase {
    private final IntakeSubsystem _intake;

    public LeftStopCommand(IntakeSubsystem intake) {
        _intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        _intake.retractLeftIntake();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}

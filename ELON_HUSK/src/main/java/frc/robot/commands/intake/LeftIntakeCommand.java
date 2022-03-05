package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class LeftIntakeCommand extends InstantCommand {
    private final IntakeSubsystem _intake;

    public LeftIntakeCommand(IntakeSubsystem intake) {
        _intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        _intake.extendLeftIntake();
    }

    @Override
    public void end(boolean interrupted) { 
    }
}

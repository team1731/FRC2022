package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class LeftIntakeJoyconCommand extends InstantCommand {
    private final IntakeSubsystem _intake;

    public LeftIntakeJoyconCommand(IntakeSubsystem intake) {
        _intake = intake;
    }

    @Override
    public void initialize() {
        _intake.extendLeftIntake();
    }

    @Override
    public void end(boolean interrupted) {
        _intake.retractLeftIntake();
    }
}

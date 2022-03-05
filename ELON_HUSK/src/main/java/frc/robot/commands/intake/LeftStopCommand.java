package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class LeftStopCommand extends InstantCommand {
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

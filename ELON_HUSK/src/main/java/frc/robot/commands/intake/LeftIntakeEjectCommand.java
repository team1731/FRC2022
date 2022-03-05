package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class LeftIntakeEjectCommand extends InstantCommand {
    private final IntakeSubsystem _intake;

    public LeftIntakeEjectCommand(IntakeSubsystem intake) {
        _intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        _intake.extendLeftEject();
    }

    @Override
    public void end(boolean interrupted) { 
        _intake.retractLeftIntake();
    }
}

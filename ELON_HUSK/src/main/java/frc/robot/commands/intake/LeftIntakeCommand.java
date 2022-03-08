package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class LeftIntakeCommand extends CommandBase {
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
    public void execute() {
        _intake.extendLeftIntake();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
	public boolean isFinished() {
		return true;
	}
}

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RightIntakeJoyconCommand extends CommandBase {
    private final IntakeSubsystem _intake;

    public RightIntakeJoyconCommand(IntakeSubsystem intake) {
        _intake = intake;
    }

    @Override
    public void initialize() {
        _intake.extendRightIntake();
    }

    @Override
    public void end(boolean interrupted) {
        _intake.retractRightIntake();
    }
}

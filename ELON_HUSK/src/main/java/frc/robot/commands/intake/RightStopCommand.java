package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class RightStopCommand extends InstantCommand {
    private final IntakeSubsystem _intake;

    public RightStopCommand(IntakeSubsystem intake) {
        _intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        _intake.retractRightIntake();
    }

    @Override
    public void end(boolean interrupted) {   
    }
}

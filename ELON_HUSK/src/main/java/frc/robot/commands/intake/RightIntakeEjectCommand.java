package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RightIntakeEjectCommand extends CommandBase {
    private final IntakeSubsystem _intake;

    public RightIntakeEjectCommand(IntakeSubsystem intake) {
        _intake = intake;
    }

    @Override
    public void initialize() {
        _intake.extendRightEject();
    }

    @Override
    public void end(boolean interrupted) { 
        _intake.retractRightIntake();
    }
}

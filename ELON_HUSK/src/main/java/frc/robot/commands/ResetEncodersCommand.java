package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class ResetEncodersCommand extends InstantCommand {
    private final DriveSubsystem _drive;

    public ResetEncodersCommand(DriveSubsystem drive){
        _drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        _drive.resetEncoders();
        System.out.println("Encoders reset by user");
    }
}

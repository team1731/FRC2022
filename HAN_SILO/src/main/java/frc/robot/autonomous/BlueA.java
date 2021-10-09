package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class BlueA extends GalacticConfiguration {
    
    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(0.10625, -0.23750),
            new Translation2d(-0.7156, -0.18333),
            new Translation2d(-0.3250, -0.12083)
        };
    }

}

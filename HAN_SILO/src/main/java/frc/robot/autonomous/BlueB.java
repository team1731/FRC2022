package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class BlueB extends GalacticConfiguration {
    
    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(0.956, -0.00833),
            new Translation2d(-0.2125, 0.0874999),
            new Translation2d(0.10625, 0.1916666)
        };
    }

}

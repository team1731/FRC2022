package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class RedA extends GalacticConfiguration {
    
    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(0.14606, -0.71440),
            new Translation2d(-0.42500, -0.32083),
            new Translation2d(-0.62187, -0.23333)
        };
    }

}

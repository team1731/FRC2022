package frc.robot.autonomous;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class BlueA extends GalacticConfiguration {
    
    @Override
    public Translation2d[] getBallPositions(){
        return new Translation2d[] {
            new Translation2d(0.0437, -0.01),
            new Translation2d(-0.949, 0.075),
            new Translation2d(-0.474, 0.158)
        };
    }

}

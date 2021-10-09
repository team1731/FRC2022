/*
    Limelight 2+ Specs:

    RES: 320 x 240 pixels
    FOV: 59.6 x 49.7 degrees
*/

package frc.robot.subsystems;

import frc.robot.Constants.AutoConstants;
import frc.robot.autonomous.BlueA;
import frc.robot.autonomous.BlueB;
import frc.robot.autonomous.RedA;
import frc.robot.autonomous.RedB;
import frc.robot.autonomous.GalacticConfiguration;
import frc.robot.vision.LimeTargetInfo;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem stores the last target coordinates and allows for easy control over the LED
 */
public class LimeLightSubsystem extends SubsystemBase {

    /**
     * The table that contains all controls and outputs for the Limelight
     */
    private NetworkTable limeTable;
    /**
     * The current vision pipeline the Limelight is set to
     */
    private NetworkTableEntry limePipeline;
    /**
     * How off the X axis (target coordinates) the target is from the crosshair in degrees
     */
    private NetworkTableEntry limeTX;
    /**
     * How off the Y axis (target coordinates) the target is from the crosshair in degrees
     */
    private NetworkTableEntry limeTY;
    /**
     * How much of the image the target covers (0%-100%)
     */
    private NetworkTableEntry limeArea;
    /**
     * The horizontal (width) sidelength of the rough bounding box (0-320 pixels)
     */
    private NetworkTableEntry limeHoriz;
    /**
     * The vertical (length) sidelength of the rough bounding box (0-320 pixels)
     */
    private NetworkTableEntry limeVert;
    /**
     * Indicates if the Limelight has a target. 1 for yes, 0 for no.
     */
    private NetworkTableEntry limeValidTargets;
    /**
     * Control for LED. 1 is off, 2 is blink, 3 is on, and 0 is default for pipeline
     */
    private NetworkTableEntry limeLED;

    private NetworkTableEntry[] limeRawX = new NetworkTableEntry[3];
    private NetworkTableEntry[] limeRawY = new NetworkTableEntry[3];
    private NetworkTableEntry[] limeRawArea = new NetworkTableEntry[3];

    /**
     * The last target that was reported by the Limelight
     */
    private LimeTargetInfo[] lastTarget = new LimeTargetInfo[] {
        LimeTargetInfo.empty,
        LimeTargetInfo.empty,
        LimeTargetInfo.empty
    };

    /**
     * Keeps track of how many systems are requesting the LED. Each system should be turning off the LED when they are done.
     * e.g. VisionRotateCommand turns the LED on while the command is active and then turns it off when deactivated.
     */
    private int ledQueries = 0;

    /**
     * The different configured pipelines on the Limelight
     */
    public enum DetectionMode {
        PowerPort,
        PowerCells
    }

    /**
     * The current Limelight pipeline
     */
    private DetectionMode currDetectionMode = DetectionMode.PowerPort;

    public LimeLightSubsystem() {
        //Set tables for easy getting
        limeTable = NetworkTableInstance.getDefault().getTable("limelight");
        limePipeline = limeTable.getEntry("pipeline");
        limeTX = limeTable.getEntry("tx");
        limeTY = limeTable.getEntry("ty");
        limeArea = limeTable.getEntry("ta");
        limeHoriz = limeTable.getEntry("thor");
        limeVert = limeTable.getEntry("tvert");
        limeValidTargets = limeTable.getEntry("tv");
        limeLED = limeTable.getEntry("ledMode");
        for(int i = 0; i < 3; i++){
            limeRawX[i] = limeTable.getEntry("tx"+i);
            limeRawY[i] = limeTable.getEntry("ty"+i);
            limeRawArea[i] = limeTable.getEntry("ta"+i);
        }

        SetDetectionMode(DetectionMode.PowerCells);

        //Keep the light off so we don't blind unfortunate spectators
        disableLED(false);

        SmartDashboard.putString("SelectedGalactic", "None");
    }

    @Override
    public void periodic() {
        //Report target when one is valid
        if(hasTarget()){
            if(currDetectionMode == DetectionMode.PowerPort){
                lastTarget[0] = new LimeTargetInfo(limeTX.getDouble(0), limeTY.getDouble(0), 
                                                limeArea.getDouble(0), limeVert.getDouble(0), limeHoriz.getDouble(0), 
                                                Timer.getFPGATimestamp());
            } else if(currDetectionMode == DetectionMode.PowerCells){
                for(int i = 0; i < lastTarget.length; i++){
                    lastTarget[i] = new LimeTargetInfo(limeRawX[i].getDouble(0), limeRawY[i].getDouble(0), 
                                                        limeRawArea[i].getDouble(0), 0, 0,
                                                        Timer.getFPGATimestamp());
                }
            }
        }

        UpdateSmartDashboard();
    }

    /**
     * Updates the Vis_HasTarget and Vis_TargetPos SmartDashboard entries
     */
    private void UpdateSmartDashboard(){
        String autoCode = SmartDashboard.getString("AUTO CODE", AutoConstants.kDEFAULT_AUTO_CODE).toUpperCase().strip();
        if(autoCode.equals("H0")){
            SetDetectionMode(DetectionMode.PowerCells);
        } else {
            SetDetectionMode(DetectionMode.PowerPort);
        }
        SmartDashboard.putBoolean("Vis_HasTarget", hasTarget());
        SmartDashboard.putString("Vis_TargetPos", hasTarget() ? lastTarget[0].getY()+", "+lastTarget[0].getZ() 
                                                                : "N/A");
    }

    /**
     * Gets the last target reported by the Limelight
     * @return The last target reported by the Limelight
     */
    public LimeTargetInfo getLastPortPos(){
        return lastTarget[0];
    }

    /**
     * Checks if the Limelight has any valid targets
     * @return Whether or not the Limelight has any valid targets
     */
    public boolean hasTarget(){
        return limeValidTargets.getDouble(0) > 0;
    }

    /**
     * Sets the Limelight pipeline
     */
    public void SetDetectionMode(DetectionMode detectionMode){
        currDetectionMode = detectionMode;

        switch(detectionMode){
            case PowerPort:
                limePipeline.setNumber(2);
                break;
            case PowerCells:
                limePipeline.setNumber(1);
                break;
        }
    }

    /**
     * Turns on the LED
     */
    public void enableLED(){
        limeLED.setNumber(3);
        ledQueries++;
    }

    /**
     * Notes that your system is done with the LED. If all systems are done with the LED, it is turned off
     */
    public void disableLED(){
        disableLED(true);
    }

    /**
     * Notes that your system is done with the LED. If all systems are done with the LED, it is turned off
     * @see disableLED()
     * @param trackQuery Whether or not to actually track the system that queried the disable. Setting to false typically forces the LED to turn off.
     */
    private void disableLED(boolean trackQuery){
        if(trackQuery){
            ledQueries--;
        }

        if(ledQueries <= 0 && trackQuery){
            limeLED.setNumber(0);
        }
    }

    private double getDistance(Translation2d pos1, Translation2d pos2){
        return Math.sqrt(
          Math.pow(pos2.getX() - pos1.getX(), 2) + Math.pow(pos2.getY() - pos1.getY(), 2)
        );
      }
    
    private double getDistance(Translation2d pos1, LimeTargetInfo targetPos){
        Translation2d pos2 = new Translation2d(targetPos.getY(), targetPos.getZ());
        return getDistance(pos1, pos2);
    }

    public int getFieldOrientation(){

        GalacticConfiguration[] modes = new GalacticConfiguration[] {
            new RedA(),
            new RedB(),
            new BlueA(),
            new BlueB(),
        };

        double[] storedDifference = new double[modes.length];

        for(int m = 0; m < modes.length; m++){
            Translation2d[] positions = modes[m].getBallPositions();
            for(int p = 0; p < positions.length; p++) {
            storedDifference[m] += getDistance(positions[p], lastTarget[p]);
            }
        }

        int minIndex = 0;
        for(int i = 0; i < storedDifference.length; i++){
            if(storedDifference[i] < storedDifference[minIndex]){
            minIndex = i;
            }
        }

        return minIndex;
        
        /*
            Algorithm that Patrick, Christian, and David worked on
        
        
        import java.util.Random;
        
        class positionAlgorithm{
            public static void main(String[] args) {
                Random rand = new Random();
                
                double[][] positionZero = { {1.0,2.0,3.0} , {5.0,6.0,7.0} };
                double[][] positionOne = { {19.0,47.0,34.0} , {23.0,43.0,12.0} };
                double[][] positionTwo = { {21.0,25.0,43.0} , {37.0,23.0,23.0} };
                double[][] positionThree = { {12.0,32.0,43.0} , {23.0,43.0,34.0} };
            double[][] randPosition = { {rand.nextInt(50),rand.nextInt(50),rand.nextInt(50)} , {rand.nextInt(50),rand.nextInt(50),rand.nextInt(50)} };
            //double[][] randPosition = { {19.0,47.0,34.0} , {23.0,43.0,12.0} };
        
            double[] storedDifference = new double [4]; 
                
                for (int i = 1; i < 5 ; i++)
                {
                    switch (i){
                        case 1: 
                            for(int j = 0; j < 2; j++)
                            {
                                storedDifference[0] += difference(randPosition[0][j], positionZero[0][j], randPosition[1][j], positionZero[1][j]);
                            }
                        break;
        
                        case 2: 
                            for(int j = 0; j < 2; j++)
                            {
                                storedDifference[1] += difference(randPosition[0][j], positionOne[0][j], randPosition[1][j], positionOne[1][j]);
                            }
                        break;
        
                        case 3: 
                            for(int j = 0; j < 2; j++)
                            {
                                storedDifference[2] += difference(randPosition[0][j], positionTwo[0][j], randPosition[1][j], positionTwo[1][j]);
                            }
                        break;
        
                        case 4: 
                            for(int j = 0; j < 2; j++)
                            {
                                storedDifference[3] += difference(randPosition[0][j], positionThree[0][j], randPosition[1][j], positionThree[1][j]);
                            }
                        break;
                    }
                }
        
                System.out.println(storedDifference[0]);
                double Min = Double.min (Double.min(storedDifference[0], storedDifference[1]), Double.min(storedDifference[2], storedDifference[3]));
                int position = 0;
        
                if (Min == storedDifference[0]){
                    position = 1;
                } else if (Min == storedDifference[1]){
                    position = 2;
                } else if (Min == storedDifference[2]){
                    position = 3;
                } else if (Min == storedDifference[3]){
                    position = 4;
                }
        
                System.out.println(position);
        
            }
        
            private static double difference (double xOne, double xTwo, double yOne, double yTwo){
                double differencePosition = Math.sqrt(Math.pow((xTwo - xOne), 2.0) + Math.pow((yTwo - yOne), 2.0) );
                return differencePosition;
            } 
        }
            */
    }
}
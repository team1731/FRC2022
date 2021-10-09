package frc.robot.vision;

public class LimeTargetInfo {

    private double x = 1.0;
    private double y;
    private double z;
    private double timestamp;

    private double area;
    private double boxLength;
    private double boxWidth;

    public static LimeTargetInfo empty = new LimeTargetInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public LimeTargetInfo(double y, double z, double area, double hor, double vert, double timestamp){
        this.y = y;
        this.z = z;
        this.timestamp = timestamp;

        this.area = area;
        this.boxLength = vert;
        this.boxWidth = hor;
    }

    public LimeTargetInfo(double y, double z){
        this.y = y;
        this.z = z;
    }

    /**
     * Returns the X of the target in robot coordinates (Z in target coordinates).
     * This is locked at 1.
     * @return
     */
    public double getX(){
        return x;
    }

    /**
     * Returns the Y of the target in robot coordinates (X in target coordinates).
     * @return
     */
    public double getY(){
        return y;
    }

    /**
     * Returns the Z of the target in robot coordinates (Y in target coordinates).
     * @return
     */
    public double getZ(){
        return z;
    }

    /**
     * Returns the timestamp when this data was captured
     * @return
     */
    public double getTimeCaptured(){
        return timestamp;
    }

    /**
     * Returns the area of the bounding box around the target in percentage of image (0-100%). Camera resolution is 320x240.
     * @return
     */
    public double getArea(){
        return area;
    }

    /**
     * Returns the length of the bounding box around the target in pixels.
     * @return
     */
    public double getLength(){
        return boxLength;
    }

    /**
     * Returns the width of the bounding box around the target in pixels.
     * @return
     */
    public double getWidth(){
        return boxWidth;
    }

}
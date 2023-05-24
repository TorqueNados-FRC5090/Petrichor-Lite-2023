package frc.robot.misc_subclasses;

// Imports
import edu.wpi.first.networktables.NetworkTableInstance;
import static frc.robot.Constants.LimelightConstants.*;

public class Limelight {

    /** Tracks whether there is a valid target */
    private double tv;
    /** Tracks rotation angle on the x-axis from limelight to target */
    private double tx;
    /** Difference in y-axis between limelight and target */
    private double a2;
    /** Tracks target area */
    private double ta;

    public boolean hasValidTarget = false;
    private double distance;
    public double driveCommand;
    public double steerCommand;

    /** Constructs a Limelight object */ 
    public Limelight() {}

    // Accessor methods
    public boolean hasTarget() { return this.hasValidTarget; }
    public double getDistance() { return this.distance; }
    public double getSteer() { return this.steerCommand; }
    public double getDrive() { return this.driveCommand; }
    public double getRotationAngle() { return this.tx; }

    /** Calculates each of limelight's values and updates their corresponding variables */
    public void updateLimelightTracking()
    {
        // Update Limelight tracking values
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        a2 = Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));

        // Stops the function here if there is no valid target
        if ( tv < 1 ) {
            hasValidTarget = false;
            driveCommand = 0;
            steerCommand = 0;
            return;
        }
        hasValidTarget = true;

        // Update steer and drive commands
        steerCommand = tx * STEER_K;
        driveCommand = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // Enforces speed limit
        if( driveCommand > MAX_DRIVE )
            driveCommand = MAX_DRIVE;

        // Calculates horizontal distance to the target
        distance = Math.round((
            TARGET_HEIGHT - LIME_HEIGHT) / 
            Math.tan(LIME_ANGLE + a2));
    }
}

package frc.robot;

// Import Constants
import frc.robot.Constants.ArmConstants.ArmState;

// Camera imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

// Subsystem and subclass imports
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.PlayerIndicator;
import frc.robot.misc_subclasses.Dashboard;
import frc.robot.misc_subclasses.Limelight;

// Command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// Misc imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    // Commands
    private Command autonCommand;

    // Subsystem and subclass objects
    private Arm arm;
    private Claw claw;
    public PlayerIndicator indicator;
    private Dashboard dashboard;
    private Limelight limelight;
    
    // Other objects
    private XboxController operatorController;
    private Compressor compressor;

    // This function is run when the robot is first started up and should be used
    // for any initialization code.
    @Override
    public void robotInit() {
        // Start the camera feed
        CameraServer.startAutomaticCapture();

        // Construct objects
        robotContainer = new RobotContainer();
        operatorController = robotContainer.getOperatorController();
        arm = robotContainer.getArm();
        claw = robotContainer.getClaw();
        indicator = robotContainer.getPlayerIndicator();
        limelight = new Limelight();
        dashboard = new Dashboard();
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    }

    // This function is called once at the start of auton
    @Override
    public void autonomousInit() {
        // Get the command to be used in auton
        autonCommand = robotContainer.getAutonomousCommand();
        // Schedule the command if there is one
        if (autonCommand != null)
            autonCommand.schedule();
    }

    // This function is called every 20ms during auton
    @Override
    public void autonomousPeriodic() { }
    
    // This function is called once at the start of teleop
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous command stops when teleop starts
        if (autonCommand != null)
            autonCommand.cancel();
        
        // Start the compressor
        compressor.enableDigital();
        arm.setTarget(ArmState.ZERO);
    }

    // This function is called every 20ms during teleop
    @Override
    public void teleopPeriodic() {
        robotContainer.teleopPeriodic();

        /*     ___________________________
         *    |  __0__                    |
         *    | |  |  |  (1) [3] [4] (5)  |
         *    | |_/_\_|  (2) [6] [x] (x)  |
         *    |___________________________|
         * 
         *    1 - Pickup human
         *    2 - Pickup floor
         * 
         *    3 - Dropoff low / Intermediate
         *    4 - Dropoff medium
         *    5 - Dropoff high
         *    
         *    6 - Zero
         */

         
        // Pressing Right bumper makes the arm go to the preset of the drop of high position
        if (operatorController.getRawButtonPressed(5))
            arm.setTarget(ArmState.DROPOFF_HIGH);

        // Pressing Y makes the arm go to the preset of the drop of medium position
        if (operatorController.getRawButtonPressed(7))
            arm.setTarget(ArmState.DROPOFF_MED);

        // Pressing Left trigger makes the arm go to the preset of the floor pickup position
        if (operatorController.getRawButtonPressed(3))
            arm.setTarget(ArmState.INTERMEDIATE);

        // Pressing A makes the arm go to the preset of the zero position
        if (operatorController.getRawButtonPressed(1))
            arm.setTarget(ArmState.ZERO);

        // Pressing Left bumper makes the arm go to the preset of the human pickup position
        if (operatorController.getRawButtonPressed(4))
            arm.setTarget(ArmState.PICKUP_HUMAN);
        
        // Pressing Left trigger makes the arm go to the preset of the floor pickup position
        if (operatorController.getRawButtonPressed(2))
            arm.setTarget(ArmState.PICKUP_FLOOR);

        // Press B to place a cone on a peg
        if(operatorController.getRawButtonPressed(6))
            arm.setTarget(ArmState.PLACE_HIGH);

       

        
    }

    // This function is called every 20ms while the robot is enabled
    @Override
    public void robotPeriodic() {    
        // Print data to the dashboard
        dashboard.printLimelightData(limelight);
        dashboard.printBasicDrivetrainData(robotContainer.getDrivetrain());
        dashboard.printIndicatorState(indicator);

        // Run any functions that always need to be running
        limelight.updateLimelightTracking();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() {

        /*     ___________________________
         *    |  __0__                    |
         *    | |  |  |  (1) [3] [4] (7)  |
         *    | |_/_\_|  (2) [5] [6] (8)  |
         *    |___________________________|
         * 
         *    1 and 2 control claw
         *    3 and 4 control telescope
         *    5 and 6 control slider
         *    7 and 8 control rotation
         */
        
        if (operatorController.getLeftTriggerAxis() > .5)
            claw.open();
        else if (operatorController.getRawButton(1))
            claw.close();
            
        if(operatorController.getRawButton(5))
            arm.getTelescopeMotor().set(.15);
        else if(operatorController.getRawButton(3))
            arm.getTelescopeMotor().set(-.3);
        else
            arm.getTelescopeMotor().set(0); 

        if(operatorController.getRawButton(2))
            arm.getSliderMotor().set(.15);
        else if(operatorController.getRawButton(4))
            arm.getSliderMotor().set(-.15);
        else
            arm.getSliderMotor().set(0); 

        if(operatorController.getRawButton(6))
            arm.getRotationMotor().set(.1);
        else if(operatorController.getRightTriggerAxis() > .5)
            arm.getRotationMotor().set(-.1);
        else
            arm.getRotationMotor().set(0); 

        if(operatorController.getRawButtonPressed(7))
            indicator.indicatorToggle();
    }
}
package frc.robot;

// Import constants
import static frc.robot.Constants.ControllerPorts.*;
import static frc.robot.Constants.ArmIDs.*;

// Command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.commands.AutonContainer;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveWithHeading;
import frc.robot.commands.LockDrivetrain;
import frc.robot.commands.GoToArmPreset;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

// Other imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Handles everything command based */
public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Arm arm = new Arm(ROTATION_ID);
    private final AutonContainer auton = new AutonContainer(drivetrain, arm);
    private final XboxController driverController = new XboxController(DRIVER_PORT);
    private final XboxController operatorController = new XboxController(OPERATOR_PORT);
    private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    /** Constructs a RobotContainer */
    public RobotContainer() {
        initChooser();

        // If the drivetrain is not busy, drive using joysticks
        drivetrain.setDefaultCommand(
            new DriveCommand(drivetrain, 
            () -> driverController.getLeftX(), 
            () -> driverController.getLeftY(),
            () -> driverController.getRightX())
        );

        configureDriverBindings();
        configureOperatorBindings();
    }

    /** Initialize the auton selector on the dashboard */
    private void initChooser() {
        autonChooser.setDefaultOption("Do Nothing", new WaitCommand(0));

        SmartDashboard.putData("Auton Selector", autonChooser);
        autonChooser.setDefaultOption("Drop High Auto", auton.dropHigh());
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // For testing
        //return auton.testAuto(testAutonChooser.getSelected(), 1, 1);

        return autonChooser.getSelected();
    }

    /** @return The robot's drivetrain */
    public Drivetrain getDrivetrain() { return drivetrain; }
    public XboxController getOperatorController() { return operatorController; }
    public Arm getArm() {return arm;}


    public void configureDriverBindings() {
        /*
         * 
         */


        Trigger toggleOrientationBtn = new Trigger(() -> driverController.getStartButton());
        toggleOrientationBtn.onTrue(new InstantCommand(() -> drivetrain.toggleFieldCentric()));

        Trigger resetHeadingBtn = new Trigger(() -> driverController.getBackButton());
        resetHeadingBtn.onTrue(new InstantCommand(() -> drivetrain.resetHeading()));

        Trigger lockBtn = new Trigger(() -> driverController.getXButton());
        lockBtn.whileTrue(new LockDrivetrain(drivetrain));

        Trigger lockHeadingZeroBtn = new Trigger(() -> driverController.getRightTriggerAxis() > .2);
        lockHeadingZeroBtn.whileTrue(
            new DriveWithHeading(drivetrain, 
            () -> driverController.getLeftX(), 
            () -> driverController.getLeftY(),
            0));
        
        Trigger lockHeading180Btn = new Trigger(() -> driverController.getLeftTriggerAxis() > .2);
        lockHeading180Btn.whileTrue(
            new DriveWithHeading(drivetrain, 
            () -> driverController.getLeftX(), 
            () -> driverController.getLeftY(),
            180));

        Trigger slowDriveBtn = new Trigger(() -> driverController.getLeftStickButton());
        slowDriveBtn.whileTrue(
            new DriveCommand(drivetrain, 
            () -> driverController.getLeftX() * .4, 
            () -> driverController.getLeftY() * .4,
            () -> driverController.getRightX() * .4));
    }

    public void configureOperatorBindings() {

        Trigger zeroPositionBtn = new Trigger(() -> driverController.getBButton());
        zeroPositionBtn.onTrue(new GoToArmPreset(arm, ArmState.ZERO));

        Trigger floorPickupBtn = new Trigger(() -> driverController.getAButton());
        floorPickupBtn.onTrue(new GoToArmPreset(arm, ArmState.FLOOR));

        Trigger forwardPositionBtn = new Trigger(() -> driverController.getXButton());
        forwardPositionBtn.onTrue(new GoToArmPreset(arm, ArmState.FORWARD));

        Trigger backwardPositionBtn = new Trigger(() -> driverController.getYButton());
        backwardPositionBtn.onTrue(new GoToArmPreset(arm, ArmState.BACKWARD));

    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;



/** Drives the robot */
public class DriveCommand extends CommandBase {
    // Declare variables that will be initialized by the constructor
    private final Drivetrain drivetrain;
    private final DoubleSupplier inputY;
    private final DoubleSupplier inputX;
    private final DoubleSupplier inputRot;

    /** 
     * Constructs a DriveCommand
     *  
     * @param drivetrain The robot's drivetrain
     * @param translationInputX The left/right translation instruction
     * @param translationInputY The forward/back translation instruction
     * @param rotation The rotational instruction
    */
    public DriveCommand(Drivetrain drivetrain, DoubleSupplier translationInputX, DoubleSupplier translationInputY, DoubleSupplier rotationInput) {

        // Initialize internal variables with values passed through params
        this.drivetrain = drivetrain;
        this.inputX = translationInputX;
        this.inputY = translationInputY;
        this.inputRot = rotationInput;
        
        // Tell the CommandBase that this command uses the drivetrain
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.drive(inputX.getAsDouble(), inputY.getAsDouble(), inputRot.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
}

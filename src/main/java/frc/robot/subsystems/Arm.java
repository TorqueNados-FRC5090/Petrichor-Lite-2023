package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

/** This class is used to control the robot's arm */
public class Arm extends SubsystemBase {

    // Declare motors used by the arm
    private CANSparkMax rotation;

    // Declare PID controllers to control the motors
    private ProfiledPIDController rotationPID;

    private ArmState currentState = ArmState.ZERO;
    private boolean active = false;
    
    /**
     * Constructs Arm subsystem
     * 
     * @param rotationId ID of the rotation motor
     */
     public Arm(int rotationId){

        rotation = new CANSparkMax(rotationId, MotorType.kBrushless);
        rotation.restoreFactoryDefaults();
        rotation.setInverted(true);
        rotation.setIdleMode(IdleMode.kBrake);
        rotationPID = new ProfiledPIDController(.25, 0, 0,
            new TrapezoidProfile.Constraints(150, 75));
        rotationPID.setTolerance(1);
    }

    // Getters
    public CANSparkMax getRotationMotor() { return rotation; }
    public ProfiledPIDController getRotationPid() { return rotationPID; }
    public double getRotationPos() { return rotation.getEncoder().getPosition(); }
    public boolean rotationAtTarget() { return rotationPID.atGoal(); }

    public ArmState getCurrentState() { return currentState; }

    /** Move the arm to one of its presets */
    public void setTarget(ArmState preset) {
        currentState = preset;

        switch(preset) {
            case ZERO:
                zeroPosition();
                break;
            case FORWARD:
                forwardPosition();
                break;
            case BACKWARD:
                backwardPosition();
                break;
            case FLOOR:
                floorPosition();
                break;
        }
    }

    /** @param setpoint The desired angle of the arm */
    private void setRotationSetpoint(double setpoint) { active = true; rotationPID.setGoal(setpoint); }

    private void zeroPosition() {
        setRotationSetpoint(0);
    }

    private void forwardPosition() {
        setRotationSetpoint(20);
    }

    private void backwardPosition() {
        setRotationSetpoint(-34);
    }

    private void floorPosition() {
        setRotationSetpoint(61);
    }

    @Override // Called every 20ms
    public void periodic() {
        if(active) {
            double rotationPIDOut = rotationPID.calculate(getRotationPos());
            rotation.setVoltage(rotationPIDOut * RobotController.getBatteryVoltage());
        }
    }
}

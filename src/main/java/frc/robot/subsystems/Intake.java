package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake {
    
    private CANSparkMax leaderMotor;
    private CANSparkMax followerMotor;

    public Intake(int leftID, int rightID){

        leaderMotor = new CANSparkMax(leftID, MotorType.kBrushless);
        leaderMotor.restoreFactoryDefaults();

        followerMotor = new CANSparkMax(rightID, MotorType.kBrushless);
        followerMotor.restoreFactoryDefaults();
        followerMotor.follow(leaderMotor, true);
    }

    /** Both motors are stopped */   
    public void stop(){
        leaderMotor.set(0);
    }

    /** Both motors are spinning inward */
    public void pickup(){
        leaderMotor.set(.75);
    }

    /** Both motors are spinning outward */
    public void shoot(){
        leaderMotor.set(-1);
    }

}

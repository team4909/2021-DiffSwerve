package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * IndexerSubsystem
 */
public class IndexerSubsystem extends SubsystemBase{

    //Makes the indexer motor controller
    CANSparkMax indexerMotor;

    public IndexerSubsystem(){
        indexerMotor = new CANSparkMax(11, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        
    }

    
    public void runIndexer(){
        indexerMotor.set(1);
    }

    public void stopIndexer(){
        indexerMotor.set(0);
    }

    public void reverseIndexer(){
        indexerMotor.set(-1);
    }

}
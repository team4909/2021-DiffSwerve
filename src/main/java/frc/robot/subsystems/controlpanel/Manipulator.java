package frc.robot.subsystems.controlpanel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GameData;

public class Manipulator extends SubsystemBase {
    
    private final int FLIP_PCM_CHANNEL = 0;
    private final double OUTPUT = 0.8;

    CANSparkMax spinMotor;
    GameData gameData;
    ColorSensor colorSensor;
    Solenoid flip;
    
    //Pneumatics
    Compressor c = new Compressor(0);
    

    public Manipulator(ColorSensor cs, GameData gd) {
        spinMotor = new CANSparkMax(13, MotorType.kBrushless);
        flip = new Solenoid(FLIP_PCM_CHANNEL);
        this.gameData = gd;
        this.colorSensor = cs;
    }

    @Override
    public void periodic() {
        // System.out.println(gameData.getGameDataColor());
        // System.out.println(colorSensor.getColor());

    }

    public void spinWheelForward() {
        spinMotor.set(OUTPUT);
    }

    public void spinWheelReverse() {
        spinMotor.set(-OUTPUT);
    }

    public void stopWheel() {
        spinMotor.set(0);
    }
    

    public void spinToColor(String selectedColor) {
        while (colorSensor.getColor() != selectedColor) {
            spinWheelForward();
        }
    }

    
    public void flipUp() {
        flip.set(true);
    }

    public void flipDown() {
        flip.set(false);
    }
}

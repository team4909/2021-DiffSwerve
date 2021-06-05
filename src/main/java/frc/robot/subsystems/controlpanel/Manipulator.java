package frc.robot.subsystems.controlpanel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
    
    private final int FLIP_PCM_CHANNEL = 0;
    private final int RPM = 300;

    CANSparkMax spinMotor;
    ColorSensor colorSensor;
    Solenoid flip;
    
    //Pneumatics
    Compressor c = new Compressor(0);
    

    public Manipulator(ColorSensor cs) {
        spinMotor = new CANSparkMax(13, MotorType.kBrushless);
        flip = new Solenoid(FLIP_PCM_CHANNEL);
        this.colorSensor = cs;
    }

    @Override
    public void periodic() {
        System.out.println(colorSensor.getColor());
    }

    public void spinWheelForward() {
        spinMotor.set(RPM);
    }

    public void spinWheelReverse() {
        spinMotor.set(-RPM);
    }

    public void stopWheel() {
        spinMotor.set(0);
    }
    

    // public void spinToColor(String selectedColor) {
    //     while (colorSensor.getColor() != selectedColor) {
    //         spinWheel();
    //     }
    // }

    public void flipUp() {
        flip.set(true);
    }

    public void flipDown() {
        flip.set(false);
    }
}

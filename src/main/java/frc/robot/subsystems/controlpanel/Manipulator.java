package frc.robot.subsystems.controlpanel;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Compressor;

public class Manipulator {
    
    CANSparkMax spinMotor;
    ColorSensor colorSensor;

    //Pneumatics
    Compressor c = new Compressor(0);
    

    public Manipulator() {
        colorSensor = new ColorSensor();
    }

    public void spinWheel() {
        spinMotor.set(300);
    }

    public void spinToColor(String selectedColor) {
        while (colorSensor.getColor() != selectedColor) {
            spinWheel();
        }
    }
}

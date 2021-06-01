package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private final I2C.Port i2cPort = I2C.Port.kMXP;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private ColorMatch match;

    @Override
    public void periodic() {
        //Sets the detected color to the current detected color
        Color detectedColor = colorSensor.getColor();
        //Stores the raw Infared value from the sensor
        double IR = colorSensor.getIR();

        //Puts the RGB values on Smart Dashboard
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        //Puts the IR number on Smart Dashboard
        SmartDashboard.putNumber("IR", IR);

        //Gets how close the sensor is to the sensor, larger value = closer
        int proximity = colorSensor.getProximity();
        SmartDashboard.putNumber("Proximity", proximity);
    }
}

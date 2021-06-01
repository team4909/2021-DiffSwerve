package frc.robot.subsystems.controlpanel;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
    private final I2C.Port i2cPort = I2C.Port.kMXP;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private ColorMatch colorMatch = new ColorMatch();

    // Estimations of the 4 colors
    private final Color blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private final Color green = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private final Color red = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private final Color yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);
    private String colorString;

    public ColorSensor() {
        colorMatch.addColorMatch(blue);
        colorMatch.addColorMatch(green);
        colorMatch.addColorMatch(red);
        colorMatch.addColorMatch(yellow);
    }

    @Override
    public void periodic() {
        //Sets the detected color to the current detected color
        Color detectedColor = colorSensor.getColor();

        //Puts the RGB values on Smart Dashboard
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);


        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

        if (match.color == blue){
            colorString = "Blue";
        } else if (match.color == green) {
            colorString = "Green";
        } else if (match.color == red) {
            colorString = "Red";
        } else if (match.color == yellow) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown lol";
        }

        SmartDashboard.putString("Current Color", colorString);
        SmartDashboard.putNumber("Color Match Confidence", match.confidence);
    }

    public String getColor() {
        return colorString;
    }
}

package frc.bionic;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class CSVOutput {
    // Creates a new String Builder to store values
    private StringBuilder builder;
    // Creates a new File object, so that we can create a file later on
    private File file;

    // New Network Table Entry for recording values
    private NetworkTableEntry sb_startSwitch;
    private NetworkTableEntry sb_endSwitch;
    
    public CSVOutput(){
        // Initializes the String Builder
        builder = new StringBuilder();
    }

    /**
     * 
     * @param numberOfColumns: Number of types of values you will store
     * @param values: A list of Strings, IN THE CORRECT ORDER ALWAYS
     */
    public void storeValues(int numberOfColumns, String... values){
        // Checks if the endswitch to stop recording values is on
        if(sb_endSwitch.getBoolean(false)){
            //While i is less that the array, go on, and after every iteration add by the number of columns
            for(int i = 0; i < values.length; i += numberOfColumns){
                // While j is less that the number of columns, go on, and after every iteration add by 1
                for(int j = 0; j < numberOfColumns; j++){
                    // Add the value of i + j to the string builder, with a comma
                    builder.append(values[i + j] + ",");
                }
                // Replace the last added comma with a new line, for CSV formatting purposes
                builder.replace(builder.lastIndexOf(","), builder.lastIndexOf(",") + 1, "\n");
            }
        }
    }

    /**
     * Method to create a new file based on the Hours, Minutes, and Seconds
     */
    private void createNewFile(){
        // Creates a new Date object
        Date date = new Date();
        // Sets a format, which the end string will use
        String strDateFormat = "hh:mm:ss a";
        // Converts the above format to an actual DateFormat object
        DateFormat dateFormat = new SimpleDateFormat(strDateFormat);
        // Creates a string based off of the above date
        String formattedDate = dateFormat.format(date);

        try {
            // Creates a new object (output<hh:mm:ss a>.csv)
            file = new File("output" + formattedDate + ".csv");
            // Makes an actual new file based off of the above name
            file.createNewFile();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    /**
     * Method to write with a PrintWriter to any file, with a string builder
     */
    private void writeToFile(){
        try {
            // Creates a new PrintWriter object, to write to the file
            PrintWriter pw = new PrintWriter(file);
            // Adds the stringbuilder string to the file
            pw.append(builder);
            // Flushes the stream
            pw.flush();
            // Closes the stream
            pw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void periodic(){
        syncShuffleboard();
    }

    protected void initShuffleboard(){
        
        ShuffleboardTab           tab;
        ShuffleboardLayout        layout;
    
        tab              = Shuffleboard.getTab("CSV Switch");
        layout           = tab.getLayout("Switches", BuiltInLayouts.kList);
        sb_startSwitch   = layout.add("Record Values", false).getEntry();
        sb_endSwitch     = layout.add("Stop Recording Values", false).getEntry();

        
        
    }

    void syncShuffleboard(){
        // Checks if the switch is turned on
        if(sb_startSwitch.getBoolean(false)){
            // Creates a new file based off of the time if it is
            createNewFile();
        }

        if(sb_endSwitch.getBoolean(false)){
            // Creates a new file based off of the time if it is
            writeToFile();
            //Turns off the end switch
            sb_startSwitch.setBoolean(false);
            sb_endSwitch.setBoolean(false);
        }

    }
}

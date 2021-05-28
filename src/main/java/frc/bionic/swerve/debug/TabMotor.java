package frc.bionic.swerve.debug;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.bionic.swerve.AbstractDrivetrain;
import frc.bionic.swerve.AbstractSwerveModule;
import frc.bionic.swerve.IMotor;
import frc.bionic.swerve.MotorFalcon500;

public class TabMotor {

    private AbstractDrivetrain drivetrain;

    private NetworkTableEntry sb_motorControl;
    private NetworkTableEntry sb_modLF, sb_modRF, sb_modLR, sb_modRR;
    private NetworkTableEntry sb_motorA, sb_motorB;
    private NetworkTableEntry sb_sel_motor, sb_sel_module;
    private NetworkTableEntry sb_mSP_0, sb_mSP_100, sb_mSP_500, sb_mSP_1000, sb_mSP_1500, sb_mSP_2000, sb_mSP_input,
            sb_mSP_apply;
    private NetworkTableEntry sb_g_mRPM, sb_g_mErr;
    private NetworkTableEntry sb_mpid_kP, sb_mpid_kI, sb_mpid_kIz, sb_mpid_kD, sb_mpid_kF, sb_mpid_max, sb_mpid_min,
            sb_mpid_apply;

    TabMotor(AbstractDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        int row = 0;
        var motorTab = Shuffleboard.getTab("Motor");
        {
            sb_motorControl = motorTab.add("Enable Control", false).withSize(2, 2).withPosition(0, 0)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        }

        // Module selector
        {
            var moduleSelector = motorTab.getLayout("Module Selector", BuiltInLayouts.kGrid);
            moduleSelector
                    .withProperties(Map.of("Number of columns", 2, "Number of rows", 2, "Label position", "HIDDEN"));
            moduleSelector.withSize(2, 2);
            moduleSelector.withPosition(2, row);

            sb_modLF = moduleSelector.add("LF", false).withSize(1, 1).withPosition(0, 0)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
            sb_modRF = moduleSelector.add("RF", false).withSize(1, 1).withPosition(1, 0)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
            sb_modLR = moduleSelector.add("LR", false).withSize(1, 1).withPosition(0, 1)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
            sb_modRR = moduleSelector.add("RR", false).withSize(1, 1).withPosition(1, 1)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        }
        // Motor selector
        {
            var motorSelector = motorTab.getLayout("Motor Selector", BuiltInLayouts.kGrid);
            motorSelector
                    .withProperties(Map.of("Number of columns", 2, "Number of rows", 1, "Label position", "HIDDEN"));
            motorSelector.withSize(2, 2);
            motorSelector.withPosition(4, row);

            sb_motorA = motorSelector.add("A", false).withSize(1, 1).withPosition(0, 0)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
            sb_motorB = motorSelector.add("B", false).withSize(1, 1).withPosition(1, 0)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        }
        // Selected Indicator
        {
            var selected = motorTab.getLayout("Selected", BuiltInLayouts.kGrid);
            selected.withProperties(Map.of("Number of columns", 2, "Number of rows", 1));
            selected.withSize(2, 2);
            selected.withPosition(6, row);

            sb_sel_module = selected.add("Mod", "").withSize(1, 1).withPosition(0, 0).getEntry();
            sb_sel_motor = selected.add("Motor", "").withSize(1, 1).withPosition(1, 0).getEntry();
        }
        // setpoints
        {
            // 0, 100, 500, 1000, 1500, 2000, text, apply
            var setpoints = motorTab.getLayout("Setpoints", BuiltInLayouts.kGrid);
            setpoints.withProperties(Map.of("Number of columns", 4, "Number of rows", 2, "Label position", "HIDDEN"));
            setpoints.withSize(6, 2);
            setpoints.withPosition(8, row);

            sb_mSP_0 = setpoints.add("0", false).withSize(1, 1).withPosition(0, 0)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
            sb_mSP_100 = setpoints.add("100", false).withSize(1, 1).withPosition(1, 0)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
            sb_mSP_500 = setpoints.add("500", false).withSize(1, 1).withPosition(2, 0)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
            sb_mSP_1000 = setpoints.add("1000", false).withSize(1, 1).withPosition(3, 0)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();

            sb_mSP_1500 = setpoints.add("1500", false).withSize(1, 1).withPosition(0, 1)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
            sb_mSP_2000 = setpoints.add("2000", false).withSize(1, 1).withPosition(1, 1)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
            sb_mSP_input = setpoints.add("input", 3000).withSize(1, 1).withPosition(2, 1).getEntry();
            sb_mSP_apply = setpoints.add("Apply", false).withSize(1, 1).withPosition(3, 1)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        }
        row = 2;
        // Graphs
        {
            sb_g_mRPM = motorTab.add("Motor RPM", 0).withSize(6, 5).withPosition(0, row)
                    .withWidget(BuiltInWidgets.kGraph).getEntry();
            sb_g_mErr = motorTab.add("Motor Error", 0).withSize(6, 5).withPosition(6, row)
                    .withWidget(BuiltInWidgets.kGraph).getEntry();
        }

        // motor PID
        {
            // kP, kI, kIz, kD, kF, max, min, apply
            var motorPidOuter = motorTab.getLayout("Motor PID", BuiltInLayouts.kGrid);
            motorPidOuter
                    .withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "HIDDEN"));
            motorPidOuter.withSize(3, 5);
            motorPidOuter.withPosition(12, row);

            sb_mpid_apply = motorPidOuter.add("Apply", false).withSize(1, 1).withPosition(0, 0)
                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();

            var motorPid = motorPidOuter.getLayout("Inner", BuiltInLayouts.kGrid);
            motorPid.withProperties(Map.of("Number of columns", 2, "Number of rows", 4));
            motorPid.withPosition(0, 1);

            sb_mpid_kP  = motorPid.addPersistent("kP",  MotorFalcon500.kMotorP ).withSize(1, 1).withPosition(0, 0).getEntry();
            sb_mpid_kI  = motorPid.addPersistent("kI",  MotorFalcon500.kMotorI ).withSize(1, 1).withPosition(0, 1).getEntry();
            sb_mpid_kIz = motorPid.addPersistent("kIz", MotorFalcon500.kMotorIz).withSize(1, 1).withPosition(0, 2).getEntry();
            sb_mpid_kD  = motorPid.addPersistent("kD",  MotorFalcon500.kMotorD ).withSize(1, 1).withPosition(0, 3).getEntry();

            sb_mpid_kF = motorPid.add("kF",   MotorFalcon500.kMotorFf).withSize(1, 1).withPosition(1, 0).getEntry();
            sb_mpid_max = motorPid.add("max", 0).withSize(1, 1).withPosition(1, 1).getEntry();
            sb_mpid_min = motorPid.add("min", 0).withSize(1, 1).withPosition(1, 2).getEntry();
        }
    }

    void perodic() {

        AbstractSwerveModule selectedModule = null;
        if (sb_modLF.getBoolean(false)) {
            sb_modLF.setBoolean(false);
            sb_sel_module.setString("LF");
        }
        if (sb_modRF.getBoolean(false)) {
            sb_modRF.setBoolean(false);
            sb_sel_module.setString("RF");
        }
        if (sb_modLR.getBoolean(false)) {
            sb_modLR.setBoolean(false);
            sb_sel_module.setString("LR");
        }
        if (sb_modRR.getBoolean(false)) {
            sb_modRR.setBoolean(false);
            sb_sel_module.setString("RR");
        }
        switch (sb_sel_module.getString("")) {
            case "LF":
                selectedModule = drivetrain.swerveLF;
                break;
            case "RF":
                selectedModule = drivetrain.swerveRF;
                break;
            case "LR":
                selectedModule = drivetrain.swerveLR;
                break;
            case "RR":
                selectedModule = drivetrain.swerveRR;
                break;
        }
        if (selectedModule == null) {
            // waiting for user to make selection
            // System.out.println("Selected Module is NULL");
            return;
        }

        IMotor selectedMotor = null;
        if (sb_motorA.getBoolean(false)) {
            sb_motorA.setBoolean(false);
            sb_sel_motor.setString("A");
            selectedMotor = selectedModule.motorA;
        }
        if (sb_motorB.getBoolean(false)) {
            sb_motorB.setBoolean(false);
            sb_sel_motor.setString("B");
            selectedMotor = selectedModule.motorB;
        }
        switch (sb_sel_motor.getString("")) {
            case "A":
                selectedMotor = selectedModule.motorA;
                break;
            case "B":
                selectedMotor = selectedModule.motorB;
                break;
        }
        if (selectedMotor == null) {
            // waiting for user to make selection
            // System.out.println("Selected MOTOR is NULL");
            return;
        }
        // both module and motor selected... now to process the inputs

        // update the graphs
        sb_g_mRPM.setDouble(selectedMotor.getVelocityRPM());
        sb_g_mErr.setDouble(selectedMotor.getClosedLoopError());

        // setpoints
        if (sb_mSP_0.getBoolean(false)) {
            sb_mSP_0.setBoolean(false); // make momentary
            selectedMotor.setGoalRPM(0);
        }
        if (sb_mSP_100.getBoolean(false)) {
            sb_mSP_100.setBoolean(false); // make momentary
            selectedMotor.setGoalRPM(100);
        }
        if (sb_mSP_500.getBoolean(false)) {
            sb_mSP_500.setBoolean(false); // make momentary
            selectedMotor.setGoalRPM(500);
        }
        if (sb_mSP_1000.getBoolean(false)) {
            sb_mSP_1000.setBoolean(false); // make momentary
            selectedMotor.setGoalRPM(1000);
        }
        if (sb_mSP_1500.getBoolean(false)) {
            sb_mSP_1500.setBoolean(false); // make momentary
            selectedMotor.setGoalRPM(1500);
        }
        if (sb_mSP_2000.getBoolean(false)) {
            sb_mSP_2000.setBoolean(false); // make momentary
            selectedMotor.setGoalRPM(2000);
        }
        if (sb_mSP_apply.getBoolean(false)) {
            sb_mSP_apply.setBoolean(false); // make momentary
            selectedMotor.setGoalRPM(sb_mSP_input.getDouble(0));
        }

        // pid
        if (sb_mpid_apply.getBoolean(false)) {
            sb_mpid_apply.setBoolean(false);
            selectedMotor.setPIIzDF(
                sb_mpid_kP.getDouble(0), 
                sb_mpid_kI.getDouble(0), 
                sb_mpid_kIz.getDouble(0),
                sb_mpid_kD.getDouble(0), 
                sb_mpid_kF.getDouble(0));
            // selectedMotor.setOutputRange(sb_mpid_max.getDouble(0), sb_mpid_min.getDouble(0));
        }

    }

    // This should stop all swerve module code from sending commands directly to
    // motors to allow this class direct control.
    public boolean useMotorControl() {
        return sb_motorControl.getBoolean(false);
    }
}

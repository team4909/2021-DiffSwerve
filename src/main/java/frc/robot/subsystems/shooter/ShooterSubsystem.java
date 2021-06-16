package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    // Target Speed
    public double targetSpeed;

    // Shooter kF
    public double SHOOTER_kF = 0.067; 
    // Shooter kP
    public double SHOOTER_kP = 1;
    // Shooter kI
    public double SHOOTER_kI = 0;
    // Shooter kD
    public double SHOOTER_kD = 9;

    //The motor closest to the 
    WPI_TalonFX flyWheel1;
    //The motor on the bottom-right, connected to belt 
    WPI_TalonFX flyWheel2;

    SlewRateLimiter targetRate = new SlewRateLimiter(1);
    
    public ShooterSubsystem(){
        flyWheel1 = new WPI_TalonFX(9);
        flyWheel2 = new WPI_TalonFX(10);

        //Sets the inverts of the motors so that they spin in the same direction
        flyWheel1.setInverted(TalonFXInvertType.Clockwise);
        flyWheel2.setInverted(TalonFXInvertType.Clockwise);

        //Has motor2 follow motor1 so that they have the same motion
        flyWheel2.follow(flyWheel1);

        flyWheel1.configFactoryDefault();

        //Sets the PID values of the Motors
        flyWheel1.config_kF(0, SHOOTER_kF);
        flyWheel1.config_kP(0, SHOOTER_kP);
        flyWheel1.config_kI(0, SHOOTER_kI);
        flyWheel1.config_kD(0, SHOOTER_kD);

        flyWheel2.config_kF(0, SHOOTER_kF);
        flyWheel2.config_kP(0, SHOOTER_kP);
        flyWheel2.config_kI(0, SHOOTER_kI);
        flyWheel2.config_kD(0, SHOOTER_kD);        


        
        
        //Uses the following for PID tuning on Smart dashboard
        SmartDashboard.putNumber("TargetVelocity", 1000);
        SmartDashboard.putData("flyWheel1", flyWheel1);
        SmartDashboard.putData("flyWheel2", flyWheel2);

        SmartDashboard.putBoolean("UseSpeedControllerControl", false);
        SmartDashboard.putNumber("RequestedPercentOutput", 0);

        SmartDashboard.putBoolean("ApplyPID", false);
        SmartDashboard.putBoolean("UsePercentOutput", false);

        SmartDashboard.putNumber("kP", SHOOTER_kP);
        SmartDashboard.putNumber("kI", SHOOTER_kI);
        SmartDashboard.putNumber("kD", SHOOTER_kD);
        SmartDashboard.putNumber("kF", SHOOTER_kF);

        SmartDashboard.putBoolean("Enabled", false);
        
    }

    @Override
    public void periodic() {

        //Gets the current velocity of the motors and puts them on SmartDashboard 
        double vel1 = flyWheel1.getSelectedSensorVelocity(0);
        double vel2 = flyWheel2.getSelectedSensorVelocity(0);

        SmartDashboard.putNumber("MeasuredVelocity1", vel1);
        SmartDashboard.putNumber("MeasuredVelocity2", vel2);
        SmartDashboard.putNumber("MeasuredVelocityDiff", vel2-vel1);

        SmartDashboard.putNumber("FW1 Current", flyWheel1.getSupplyCurrent());
        SmartDashboard.putNumber("FW2 Current", flyWheel2.getSupplyCurrent());



        flyWheel2.follow(flyWheel1);

        // //when the "Enabled" button on SmartDashboard is false, Use the smart dashbord velocity
        // if (SmartDashboard.getBoolean("Enabled", false)) {
        //     SlewRateLimiter targetRate = new SlewRateLimiter(1.5);
        //     System.out.println(targetRate.calculate(SmartDashboard.getNumber("TargetVelocity", 0)));
        //     //flyWheel1.set(TalonFXControlMode.PercentOutput, targetRate.calculate(SmartDashboard.getNumber("TargetVelocity", 0)));
        //     flyWheel1.set(TalonFXControlMode.Velocity, SmartDashboard.getNumber("TargetVelocity", 0));
        //     //flyWheel1.set(TalonFXControlMode.PercentOutput, 1.0);
        // }


        //Sets the PID values form SmartDashboard
        SHOOTER_kP = SmartDashboard.getNumber("kp", 0);
        SHOOTER_kI = SmartDashboard.getNumber("ki", 0);
        SHOOTER_kD = SmartDashboard.getNumber("kd", 0);
        SHOOTER_kF = SmartDashboard.getNumber("kf", 0);

        //Uses the inputed PID values form Smart Dashbord and applies them
        if (SmartDashboard.getBoolean("ApplyPID", false)) {
            SmartDashboard.putBoolean("ApplyPID", false);

            flyWheel1.config_kF(0, SHOOTER_kF);
            flyWheel1.config_kP(0, SHOOTER_kP);
            flyWheel1.config_kI(0, SHOOTER_kI);
            flyWheel1.config_kD(0, SHOOTER_kD);
        }

        //If we are not enabled, sets the velocity to 0
        // if (!SmartDashboard.getBoolean("Enabled", false)) {
        //     flyWheel1.set(TalonFXControlMode.PercentOutput, 0);
        //     return;
        // }
    }

    public void runShooter(){
        // flyWheel1.set(TalonFXControlMode.Velocity, targetRate.calculate(SmartDashboard.getNumber("TargetVelocity", 0)));
        flyWheel1.set(TalonFXControlMode.Velocity, SmartDashboard.getNumber("TargetVelocity", 0));

    }
}

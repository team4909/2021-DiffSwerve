import static org.junit.Assert.*;

import org.junit.*;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.Vec2d;

public class SwerveStateTest {


  @Before // this method will run before each test
  public void setup() {
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {
  }

  @Test // marks this method as a test
  public void doesntWorkWhenClosed() {
    SwerveModuleState res;

    res = Robot.getState(Math.sqrt(2), -Math.sqrt(2));
    // System.out.println("Speed: "+res.speedMetersPerSecond);
    // System.out.println("Angle: "+res.angle.getDegrees());
    assert res.speedMetersPerSecond == 2;
    assert res.angle.getDegrees() == 0;

    res = Robot.getState(-Math.sqrt(2), Math.sqrt(2));
    // System.out.println("Speed: "+res.speedMetersPerSecond);
    // System.out.println("Angle: "+res.angle.getDegrees());
    assert res.speedMetersPerSecond == -2;
    assert res.angle.getDegrees() == 0;

    res = Robot.getState(Math.sqrt(2), Math.sqrt(2));
    // System.out.println("Speed: "+res.speedMetersPerSecond);
    // System.out.println("Angle: "+res.angle.getDegrees());
    assert res.speedMetersPerSecond == 0;
    assert res.angle.getDegrees() == 2;

    res = Robot.getState(-Math.sqrt(2), -Math.sqrt(2));
    // System.out.println("Speed: "+res.speedMetersPerSecond);
    // System.out.println("Angle: "+res.angle.getDegrees());
    assert res.speedMetersPerSecond == 0;
    assert res.angle.getDegrees() == -2;

    // final Vec2d MOTOR_1_VECTOR = new Vec2d(1/Math.sqrt(2), 1/Math.sqrt(2));

    // Vec2d vec = MOTOR_1_VECTOR.scale(5);
    // System.out.println("x: "+vec.getX());
    // System.out.println("y: "+vec.getY());
    // System.out.println("mag: "+vec.getMagnitude());
    
  }
}
import static org.junit.Assert.*;

import static org.junit.Assert.assertEquals;

import org.junit.*;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.MotorRPMs;
import frc.robot.Robot;
import frc.robot.Vec2d;

public class SwerveStateTest {


/*
  NOTE: This needs to be reimplemented. Called functions no longer exist.

  @Before // this method will run before each test
  public void setup() {
  }

  @After // this method will run after each test
  public void shutdown() throws Exception {
  }

  @Test // marks this method as a test
  public void testGetState() {
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
  }

  @Test // marks this method as a test
  public void testcalcMotorPowers() {
    MotorRPMs pwr;

    // full forward
    pwr = Robot.calcMotorPowers(new Vec2d(1, 0));
    System.out.println("A: "+pwr.a+ " B: "+pwr.b);
    assertEquals(pwr.a, -pwr.b, .0001);

    // full reverse
    pwr = Robot.calcMotorPowers(new Vec2d(-1, 0));
    System.out.println("A: "+pwr.a+ " B: "+pwr.b);
    assertEquals(pwr.a, -pwr.b, .0001);

    // full clockwise
    pwr = Robot.calcMotorPowers(new Vec2d(0, 1));
    System.out.println("A: "+pwr.a+ " B: "+pwr.b);
    assertEquals(pwr.a, pwr.b, .0001);

    // full counter-clockwise
    pwr = Robot.calcMotorPowers(new Vec2d(0, -1));
    System.out.println("A: "+pwr.a+ " B: "+pwr.b);
    assertEquals(pwr.a, pwr.b, .0001);

    // only motor a (projecting unit vector onto unit vector)
    pwr = Robot.calcMotorPowers(new Vec2d(1/Math.sqrt(2), 1/Math.sqrt(2)));
    System.out.println("A: "+pwr.a+ " B: "+pwr.b);
    assertEquals(pwr.a, 1, .0001);
    assert pwr.b == 0;


  }
*/
}

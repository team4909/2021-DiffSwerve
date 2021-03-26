/*
 * Team 4909, Bionics
 * Billerica Memorial High School
 *
 * Copyright:
 *   2021 Bionics
 *
 * License:
 *   MIT: https://opensource.org/licenses/MIT
 *   See the LICENSE file in the project's top-level directory for details.
 */

package frc.peyton;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot
{
  private PeytonSwerveModule    swerveRF; // right front
  private PeytonSwerveModule    swerveLF; // left front
  private PeytonSwerveModule    swerveLR; // left rear
  private PeytonSwerveModule    swerveRR; // right rear

  public void robotInit()
  {
    swerveRF = new PeytonSwerveModule(1, 2, 0, 1, "RF", "Peyton");
    swerveLF = new PeytonSwerveModule(3, 4, 2, 3, "LF", "Peyton");
    swerveLR = new PeytonSwerveModule(5, 6, 4, 5, "LR", "Peyton");
    swerveRR = new PeytonSwerveModule(7, 8, 6, 7, "RR", "Peyton");
  }

  public void robotPeriodic()
  {
    swerveRF.periodic();
    swerveLF.periodic();
    swerveLR.periodic();
    swerveRR.periodic();
  }

  public void teleopInit()
  {
    // For debugging, reset all encoders to zero when beginning teleop
    swerveRF.setZero();
    swerveLF.setZero();
    swerveLR.setZero();
    swerveRR.setZero();
  }
}

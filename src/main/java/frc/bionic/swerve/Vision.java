package frc.bionic.swerve;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  // The slope of the line made from the camera to the target
  private double slope;
  // The y-intercept of the line made from the limelight to the target
  private double yintercept;
  // Height that the reference target is at
  private double targetHeight;
  // The height of the limelight relative to the ground
  private double cameraHeight;
  // The angle of the limelight relative to a paralell ground in relation to the ground
  private double cameraAngle;

  // The integer value that the API uses to turn the lights on/flash/off
  private final double lightOn = 3.0;
  private final double lightFlash= 2.0;
  private final double lightOff = 1.0;

  // Establishes a Network Table feed from the limelight 
  private final NetworkTable frontCamFeed;
  private NetworkTable activeCamFeed;
  private boolean usingFrontCam = false;

  //Adds entries for shuffleboard values
  private NetworkTableEntry sb_x_offset;
  private NetworkTableEntry sb_y_offset;
  private NetworkTableEntry sb_distance_to_target;
  private NetworkTableEntry sb_target_area;
  private NetworkTableEntry sb_target_skew;
  private NetworkTableEntry sb_target_found;
  private NetworkTableEntry sb_target_acquired;

  private NetworkTableEntry sb_limelight_latency;
  private NetworkTableEntry sb_front_camera;

  // Whether the limelight has any valid targets (0 or 1)
  private double tv;
  // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
  private double tx;
  // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  private double ty;
  // Target Area (0% of image to 100% of image)
  private double ta;
  // Skew or rotation (-90 degrees to 0 degrees)
  private double ts;
  // The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
  private double tl; 

  // Front light object
  private double frontLight;
  // Back light object 
  private double backLight;
  // The value the API uses to differentiate between what the user sees
  private double pipeline = 0;

  public Vision(double slope, double yintercept, double targetHeight, double cameraHeight, double cameraAngle) {
    super();
    this.slope = slope;
    this.yintercept = yintercept;
    this.targetHeight = targetHeight;
    this.cameraHeight = cameraHeight;
    this.cameraAngle = cameraAngle;

    this.frontCamFeed = NetworkTableInstance.getDefault().getTable("limelight");
    this.setFrontCamFeed();
    this.updateTableVariables();
  }  
  
  double ledState = 1;

  @Override
  public void periodic() {
    frontCamFeed.getEntry("ledMode").setNumber(ledState);
    frontCamFeed.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Updates network frontCamFeed variables.
   */
  private void updateTableVariables() {
    tv = activeCamFeed.getEntry("tv").getDouble(0.0);
    tx = activeCamFeed.getEntry("tx").getDouble(0.0);
    ty = activeCamFeed.getEntry("ty").getDouble(0.0);
    ta = activeCamFeed.getEntry("ta").getDouble(0.0);
    ts = activeCamFeed.getEntry("ts").getDouble(0.0);
    tl = activeCamFeed.getEntry("tl").getDouble(0.0);
  }

  /**
   * Sets the led states
  */
  public void setLights(final double ledState){
    this.ledState = ledState;
  }

  /**
   * 
   * Establishes a pipeline based on what the user sees
  */
  public void setPipeline(final double pipeline){
    this.pipeline = pipeline;
  }

  /**
   * Gets the X offset in degrees.
   *
   * @return The X offset in degrees.
   */
  public double getXOffset() {
    return tx;
  }

  /**
   * Gets the Y offset in radians.
   *
   * @return The Y offset in radians.
   */
  public double getYOffset() {
    return ty;
  }

  /**
   * Gets the target area.
   *
   * @return The target area.
   */
  public double getTargetArea() {
    return ta;
  }

  /**
   * Gets the target skew.
   *
   * @return Target Skew.
   */
  public double getTargetSkew() {
    return ts;
  }

  /**
   * Gets the X limelight's latency.
   *
   * @return Latency
   */
  public double getLatency() {
    return tl;
  }

  /**
   * Returns 1 if a target is visible, returns 0 if there's no target visible.
   *
   * @return 1 or 0
   */
  public double isTargetVisibleDouble() {
    return tv;
  }

  /**
   * Gets the distance of the target using the formula from the limelight's documentation.
   *
   * @return The distance to the target using a formula.
   */
  public double calculateDistanceFromCameraHeight(final double targetHeight, final double cameraHeight, final double cameraAngle) {
    final double methodDistance = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(cameraAngle + getYOffset()));
    return methodDistance;
  }

  /**
   * Gets the distance to the target based of off its area using a slope-intercept formula.
   *
   * @return The distance based off of the area of the target.
   */
  public double calculateDistanceArea() {
    return slope * ta + yintercept;
  }

  /**
   * Sets the camera feed to stream from the front facing camera.
   */
  public void setFrontCamFeed() {
    this.activeCamFeed = this.frontCamFeed;
    this.usingFrontCam = true;
    this.frontLight = this.lightOn;
    this.backLight = this.lightOff;
    this.cameraAngle = 0;
  }

  public void flash() {
    this.frontLight = this.lightFlash;
    this.backLight = this.lightFlash;
  }

  public boolean isUsingFrontCam() {
    return this.usingFrontCam;
  }

  public boolean targetAcquired(){
    if(this.tv == 1){
        return true;
    }
    else{
        return false;
    }
  }

  /**
   * Updates the values of the Vision class.
   */

  protected void initShuffleboard(){
    ShuffleboardTab tab;
    ShuffleboardLayout targetLayout;
    ShuffleboardLayout limelightLayout;

    tab = Shuffleboard.getTab("limelight");

    targetLayout = tab.getLayout("Target", BuiltInLayouts.kGrid);
    limelightLayout = tab.getLayout("Limelight (generic)", BuiltInLayouts.kGrid);

    sb_x_offset = targetLayout.add("X-Offset", 0).getEntry();
    sb_y_offset = targetLayout.add("Y-Offset", 0).getEntry();
    sb_distance_to_target = targetLayout.add("Distance to target", 0).getEntry();
    sb_target_area = targetLayout.add("Target Area", 0).getEntry();
    sb_target_skew = targetLayout.add("Target Skew", 0).getEntry();
    sb_target_found = targetLayout.add("Target Found", 0).getEntry();
    sb_target_acquired = targetLayout.add("Target Acquired", 0).getEntry();

    sb_limelight_latency = limelightLayout.add("Limelight Latency", 0).getEntry();
    sb_front_camera = limelightLayout.add("Front Camera", 0).getEntry();

  }

  void syncShuffleboard(){
    updateTableVariables();

    sb_x_offset.setDouble(getXOffset());
    sb_y_offset.setDouble(getYOffset());
    sb_distance_to_target.setDouble(calculateDistanceFromCameraHeight(this.targetHeight,
                                                                      this.cameraHeight,
                                                                      this.cameraAngle));
    sb_target_area.setDouble(calculateDistanceArea());
    sb_target_skew.setDouble(getTargetArea());
    sb_target_found.setDouble(isTargetVisibleDouble());
    sb_target_acquired.setBoolean(this.targetAcquired());

    sb_limelight_latency.setDouble(getLatency());
    sb_front_camera.setBoolean(this.usingFrontCam);

  }

}
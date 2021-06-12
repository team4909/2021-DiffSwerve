package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class GameData {

  String gameData;

  public GameData() {
    gameData = DriverStation.getInstance().getGameSpecificMessage();
  }

  public String getGameDataColor() {
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          // Blue case code
          return "Blue";
        case 'G':
          // Green case code
          return "Green";
        case 'R':
          // Red case code
          return "Red";
        case 'Y':
          // Yellow case code
          return "Yellow";
        default:
          // This is corrupt data
          return "Corrupt";
      }
    } else {
      return "No Data Received";
    }
  }

}

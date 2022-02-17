package frc.robot.Utils;

public enum Presets {

    kMain(0.63, -0.83),
    kLow(0.2, -0.2),
    kHigh(0.8, -0.8);

    public final double botVal;
    public final double topVal;

    Presets(double _topVal, double _botVal) {
      this.topVal = _topVal;
      this.botVal = _botVal;
    }
    
}
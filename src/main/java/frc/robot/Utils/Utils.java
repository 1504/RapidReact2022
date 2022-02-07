package frc.robot.Utils;

//import org.json.simple.JSONArray;

public class Utils {
    
    public static double Clamp(double value, double min, double max) {
        if (value > max) {
            return max;
        }
        if (value < min) {
            return min;
        }
        return value;
    }
    

}

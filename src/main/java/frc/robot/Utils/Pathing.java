package frc.robot.Utils;

public class Pathing {
    
    /**
     * 
     * @param _x The x values of the points - 3 points
     * @param _y The y values of the points - 3 points
     * @return An 8x1 matrix representing the coefficients of the polynomials
     */
    public static double[][] interpolate(double[] _x, double[] _y) {
        double[][] A = {{Math.pow(_x[0], 3), Math.pow(_x[0], 2), _x[0], 1, 0, 0, 0, 0},
                          {Math.pow(_x[1], 3), Math.pow(_x[1], 2), _x[1], 1, 0, 0, 0, 0},
                          {0, 0, 0, 0, Math.pow(_x[1], 3), Math.pow(_x[1], 2), _x[1], 1},
                          {0, 0, 0, 0, Math.pow(_x[2], 3), Math.pow(_x[2], 2), _x[2], 1},
                          {3 * Math.pow(_x[1], 2), 2 * _x[1], 1, 0, -3 * Math.pow(_x[1], 2), -2 * _x[1], -1, 0},
                          {6 * _x[1], 2, 0, 0, -6 * _x[1], -2, 0, 0},
                          {6 * _x[0], 2, 0, 0, 0, 0, 0, 0},
                          {0, 0, 0, 0, 6 * _x[2], 2, 0, 0}};
        double[][] X = {{_y[0]},
                        {_y[1]},
                        {_y[1]},
                        {_y[2]},
                        {0},
                        {0},
                        {0},
                        {0}};

        //Get inverse of A and multiply by X to get the values needed

        return A;

    }

}

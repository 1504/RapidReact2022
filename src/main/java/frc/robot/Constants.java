// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class DriveConstants {
        public static final int FRONT_LEFT = 1;
        public static final int BACK_LEFT = 3;
        public static final int BACK_RIGHT = 4;
        public static final int FRONT_RIGHT = 2;

        //public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(1, 1);
    }
    public final class IOConstants {
        public static final int LEFT_JOYSTICK = 0;
        public static final int RIGHT_JOYSTICK = 1;
        public static final int SHOOT_CONTROLLER = 2;

        public static final int LED_PORT = 6;
    }
    public static final class ShootConstants {
        public static final int TOP_SHOOTER = 14;
        public static final int BOT_SHOOTER = 13;
        public static final double TOP_VALUE = 1;
        public static final double BOT_VALUE = 1;
        public static final int ELEVATOR_PORT = 41;

        public static final boolean TESTING = false;
    }
    public static final class WinchConstants {
        public static final int RIGHT_WINCH = 17;
        public static final int LEFT_WINCH = 16;
    }

}

package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.C;

@Config
public class RC_Claw {
    public static double openBottom = 0.1;
    public static double closeBottom = 0.44;
    public static double openBottomRev = 0.3;
    public static double openTopRev = 0.18;

    public static double openTop = 0.08;
    public static double closeTop = 0.33;
    public static double partialTop = 0.2;
    public static double partialBottom = 0.3;
    public static int pickupPause = 250;
    public static int dropTopPause = 400;
    public static int dropBottomPause = 600;
}
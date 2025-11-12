package org.firstinspires.ftc.teamcode.pedroPathing.otherclass;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Configablestuff {
    // 駕駛參數
    public static double SLOW_MODE_MULTIPLIER = 0.5;

    // PD 控制器參數 (即時調校)
    public static double HEADING_KP = 0.5;
    public static double HEADING_KD = 0.05;

    // PD 鎖定目標 (即時調校)
    public static double TARGET_X = 132.0;
    public static double TARGET_Y = 144.0;

    public static Pose botPose;
}

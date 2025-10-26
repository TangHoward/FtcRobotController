package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoMoveToBlock extends BlocksOpModeCompanion {
    private static double kP_pos =0.03,kI_pos= 0 , kD_pos = 0 ;
    private static double kP_head =0.015,kI_head= 0 , kD_head = 0 ;
    private static DcMotor mFL,mFR,mRR,mRL;
    private static GoBildaPinpointDriver pinPoint;

    @ExportToBlocks(
            heading = "初始需要的東西",
            tooltip = "",
            comment = "",
            parameterLabels = {},
            parameterDefaultValues = {}
    )
    public static void init(){
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mRR = hardwareMap.get(DcMotor.class, "mRR");
        mRL = hardwareMap.get(DcMotor.class, "mRL");

        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinPoint");
    }

    @ExportToBlocks(
            heading = "設定PID的數值",
            parameterLabels = {"位置P","位置I","位置D","角度P","角度I","角度D"},
            parameterDefaultValues = {"0.03","0","0","0.015","0","0"}
    )
    public static void pidetting(double p_pos,double i_pos,double d_pos,double p_head,double i_head,double d_head){
        kP_pos = p_pos;
        kI_pos = i_pos;
        kD_pos = d_pos;

        kP_head = p_head;
        kI_head = i_head;
        kD_head = d_head;
    }
    @ExportToBlocks(
            heading = "移動(公分)",
            parameterLabels = {"前後","左右","旋轉","移動允許誤差","轉動允許誤差"},
            parameterDefaultValues ={"0.0","0.0","0.0","1.0","0.1"}
    )
    public static void automove(double targetX,double targetY,double targetHeadingDeg, double positionTolerance, double headingTolerance){

        double integralX = 0, lastErrorX = 0;
        double integralY = 0, lastErrorY = 0;
        double integralHeading = 0, lastErrorHeading = 0;

        mFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (true) {

            pinPoint.update();

            double currentX = pinPoint.getPosX(DistanceUnit.CM);
            double currentY = pinPoint.getPosY(DistanceUnit.CM);
            double heading = pinPoint.getHeading(AngleUnit.DEGREES);

            // 計算位置誤差
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double distanceToTarget = Math.hypot(errorX, errorY);

            // 計算角度誤差 (包裝到 -180~180)
            double errorHeading = angleWrap(targetHeadingDeg - heading);


            if (distanceToTarget < positionTolerance && Math.abs(errorHeading) < headingTolerance) {
                stopMotors();
                break;
            }


            // PID 積分與微分計算
            integralX += errorX;
            integralY += errorY;
            integralHeading += errorHeading;

            integralX = clamp(integralX, -50, 50);
            integralY = clamp(integralY, -50, 50);
            integralHeading = clamp(integralHeading, -30, 30);

            double derivativeX = errorX - lastErrorX;
            double derivativeY = errorY - lastErrorY;
            double derivativeHeading = errorHeading - lastErrorHeading;

            lastErrorX = errorX;
            lastErrorY = errorY;
            lastErrorHeading = errorHeading;

            // PID 輸出 (目標速度，還沒限速) 看不懂的東西 反正就是一坨計算
            double targetVxGlobal = kP_pos * errorX + kI_pos * integralX + kD_pos * derivativeX;
            double targetVyGlobal = kP_pos * errorY + kI_pos * integralY + kD_pos * derivativeY;
            double targetOmega = kP_head * errorHeading + kI_head * integralHeading + kD_head * derivativeHeading;


            // 全局速度轉換成本體座標速度
            double headingRad = Math.toRadians(heading);
            double robotVx =  targetVxGlobal * Math.cos(headingRad) + targetVyGlobal * Math.sin(headingRad);
            double robotVy = -targetVxGlobal * Math.sin(headingRad) + targetVyGlobal * Math.cos(headingRad);

            // 計算四輪馬達功率

            double powerFL = robotVx - robotVy - targetOmega;
            double powerFR = robotVx + robotVy + targetOmega;
            double powerBL = robotVx + robotVy - targetOmega;
            double powerBR = robotVx - robotVy + targetOmega;


            // 限制輸出功率
            mFL.setPower(clamp(powerFL, -1, 1));
            mFR.setPower(clamp(powerFR, -1, 1));
            mRL.setPower(clamp(powerBL, -1, 1));
            mRR.setPower(clamp(powerBR, -1, 1));

            telemetry.addData("Position", "X=%.2f Y=%.2f Dist=%.2f", currentX, currentY, distanceToTarget);
            telemetry.addData("Heading", "Current=%.2f Target=%.2f Error=%.2f", heading, targetHeadingDeg, errorHeading);
            telemetry.addData("Speed", "Vx=%.3f Vy=%.3f Omega=%.3f", robotVx, robotVy, targetOmega);
            telemetry.addData("MotorsSpeed","FL=%.2f FR=%.2f RL=%.2f RR=%.2f", mFL.getPower(), mFR.getPower(), mRL.getPower(), mRR.getPower());
            telemetry.update();
        }


    }


    private static void stopMotors() {
        mFL.setPower(0);
        mFR.setPower(0);
        mRL.setPower(0);
        mRR.setPower(0);
    }

    // 解決里程計的角度問題
    private static double angleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // 就只是更方便寫程式
    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

}

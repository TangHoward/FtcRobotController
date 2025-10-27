package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoMoveToBlock extends BlocksOpModeCompanion {
    private static double kP_pos =0.03,kI_pos= 0 , kD_pos = 0 ;
    private static double kP_head =0.015,kI_head= 0/*建議在0.0001~0.001*/ , kD_head = 0 ;
    private static DcMotor mFL,mFR,mRR,mRL;
    private static GoBildaPinpointDriver pinPoint;

    private static BNO055IMU imu;
    private static Acceleration lastAceel = null;
    private static final double COLLISION_THRESHOLD_ACCEL_CHANGE = 25.0;

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

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //這裡依照各組更改
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }


    @ExportToBlocks(
            heading = "設定PID的數值",
            parameterLabels = {"位置P","位置I","位置D","角度P","角度I","角度D"},
            parameterDefaultValues = {"0.03","0","0","0.015","0","0"}
    )
    public static void PIDsetting(double p_pos,double i_pos,double d_pos,double p_head,double i_head,double d_head){
        kP_pos = p_pos;
        kI_pos = i_pos;
        kD_pos = d_pos;

        kP_head = p_head;
        kI_head = i_head;
        kD_head = d_head;
    }
    @ExportToBlocks(
            heading = "移動(公分)",
            parameterLabels = {"前後Y","左右X","旋轉(逆時針)","移動允許誤差","轉動允許誤差"},
            parameterDefaultValues ={"0.0","0.0","0.0","1.0","0.1"}
    )
    public static void Automove(double targetY,double targetX,double targetHeadingDeg, double positionTolerance, double headingTolerance){


        pinPoint.update();
        double startX = pinPoint.getPosX(DistanceUnit.CM);
        double startY = pinPoint.getPosY(DistanceUnit.CM);
        double startHeading = pinPoint.getHeading(AngleUnit.DEGREES);

        boolean reachedTarget = move(targetY, targetX, targetHeadingDeg, positionTolerance, headingTolerance, true);

        if (!linearOpMode.opModeIsActive()) {
            // 如果 OpMode 已經停止，則跳出
            StopMotors();
            return;
        }

        if(!reachedTarget){
            telemetry.addData("撞到了!!","回到原點");
            telemetry.update();

            move(startY, startX, startHeading, 2, 5, false);
        }
        StopMotors();
    }

    private static boolean move(double targetY,double targetX,double targetHeadingDeg, double positionTolerance, double headingTolerance, boolean needcareforce ) {
        double integralX = 0, lastErrorX = 0;
        double integralY = 0, lastErrorY = 0;
        double integralHeading = 0, lastErrorHeading = 0;

        mFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (linearOpMode.opModeIsActive()) {
            pinPoint.update();

            double currentX = pinPoint.getPosX(DistanceUnit.CM);
            double currentY = pinPoint.getPosY(DistanceUnit.CM);
            double heading = pinPoint.getHeading(AngleUnit.DEGREES);

            // 算機器到目標的誤差
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double distanceToTarget = Math.hypot(errorX, errorY);
            double errorHeading = AngleWrap(targetHeadingDeg - heading);


            if (CheckCollision() && needcareforce) {
                telemetry.addData("!!! 碰撞偵測 !!!", "偵測到撞擊");
                telemetry.update();
                StopMotors();
                return false;// 偵測到碰撞，立即退出移動迴圈
            }
            if (lastAceel != null && needcareforce) { // 只需要檢查 lastAceel
                Acceleration currentAccelDebug = imu.getOverallAcceleration();
                if (currentAccelDebug != null) {

                    double deltaX = Math.abs(currentAccelDebug.xAccel - lastAceel.xAccel);
                    double deltaY = Math.abs(currentAccelDebug.yAccel - lastAceel.yAccel);
                    linearOpMode.telemetry.addData("Jerk (Accel Change)", "dX=%.2f dY=%.2f (Threshold: %.1f)",
                            deltaX,
                            deltaY,
                            COLLISION_THRESHOLD_ACCEL_CHANGE);

                }
            }

            //確認誤差是否達到允許值
            if (distanceToTarget < positionTolerance && Math.abs(errorHeading) < headingTolerance) {
                StopMotors();
                return true;
            }

            // PID 積分與微分計算
            integralX += errorX;
            integralY += errorY;
            integralHeading += errorHeading;

            integralX = Clamp(integralX, -50, 50);
            integralY = Clamp(integralY, -50, 50);
            integralHeading = Clamp(integralHeading, -30, 30);

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
            double robotVx = targetVxGlobal * Math.cos(headingRad) - targetVyGlobal * Math.sin(headingRad);
            double robotVy = targetVxGlobal * Math.sin(headingRad) + targetVyGlobal * Math.cos(headingRad);

            // 計算四輪馬達功率

            double powerFL = robotVy + robotVx + targetOmega;
            double powerFR = robotVy - robotVx - targetOmega;
            double powerRL = robotVy - robotVx + targetOmega;
            double powerRR = robotVy + robotVx - targetOmega;

            mFL.setPower(Clamp(powerFL, -1, 1));
            mFR.setPower(Clamp(powerFR, -1, 1));
            mRL.setPower(Clamp(powerRL, -1, 1));
            mRR.setPower(Clamp(powerRR, -1, 1));

            telemetry.addData("Position", "X=%.2f Y=%.2f Dist=%.2f", currentX, currentY, distanceToTarget);
            telemetry.addData("Heading", "Current=%.2f Target=%.2f Error=%.2f", heading, targetHeadingDeg, errorHeading);
            telemetry.addData("Speed", "Vx=%.3f Vy=%.3f Omega=%.3f", robotVx, robotVy, targetOmega);
            telemetry.addData("MotorsSpeed", "FL=%.2f FR=%.2f RL=%.2f RR=%.2f", mFL.getPower(), mFR.getPower(), mRL.getPower(), mRR.getPower());
            telemetry.update();
        }
        return false;
    }
    private static void StopMotors() {
        mFL.setPower(0);
        mFR.setPower(0);
        mRL.setPower(0);
        mRR.setPower(0);
    }

    // 解決里程計的角度問題
    private static double AngleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // 就只是更方便寫程式
    private static double Clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
    private static boolean CheckCollision() {

        Acceleration currentAceel = imu.getOverallAcceleration();

        if (currentAceel == null) {
            return false;
        }

        if (lastAceel == null) {
            lastAceel = currentAceel;
            return false;
        }

        double deltaAccelX = Math.abs(currentAceel.xAccel - lastAceel.xAccel);
        double deltaAccelY = Math.abs(currentAceel.yAccel - lastAceel.yAccel);

        lastAceel = currentAceel;

        return deltaAccelX > COLLISION_THRESHOLD_ACCEL_CHANGE || deltaAccelY > COLLISION_THRESHOLD_ACCEL_CHANGE;
    }


}

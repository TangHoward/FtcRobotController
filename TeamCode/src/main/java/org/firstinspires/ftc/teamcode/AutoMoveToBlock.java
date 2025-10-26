package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
    private static double kP_head =0.015,kI_head= 0 , kD_head = 0 ;
    private static DcMotor mFL,mFR,mRR,mRL;
    private static GoBildaPinpointDriver pinPoint;


    private static IMU imu;
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

        imu = hardwareMap.get(IMU.class, "imu");

        // 使用已確認可編譯的構造函式 (你需要 ImuOrientationOnRobot 參數)
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        // 3. 初始化 IMU
        imu.initialize(parameters);
        imu.resetYaw();
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
    public static void automove(double targetY,double targetX,double targetHeadingDeg, double positionTolerance, double headingTolerance){

        double integralX = 0, lastErrorX = 0;
        double integralY = 0, lastErrorY = 0;
        double integralHeading = 0, lastErrorHeading = 0;

        mFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (linearOpMode.opModeIsActive()) {

            if (checkCollision()) {
                linearOpMode.telemetry.addData("!!! 碰撞偵測 !!!", "偵測到猛烈撞擊，移動停止。");
                linearOpMode.telemetry.update();
                stopMotors();
                break; // 偵測到碰撞，立即退出移動迴圈
            }
            if (lastAceel != null && imu instanceof BNO055IMU) {
                BNO055IMU bno055Imu = (BNO055IMU) imu;

                // 由於 checkCollision 中已使用 getOverallAcceleration()，這裡保持一致
                Acceleration currentAccelDebug = bno055Imu.getOverallAcceleration();

                if (currentAccelDebug != null) {
                    double deltaX = Math.abs(currentAccelDebug.xAccel - lastAceel.xAccel);
                    double deltaY = Math.abs(currentAccelDebug.yAccel - lastAceel.yAccel);
                    linearOpMode.telemetry.addData("Jerk (Accel Change)", "dX=%.2f dY=%.2f (Threshold: %.1f)",
                            deltaX,
                            deltaY,
                            COLLISION_THRESHOLD_ACCEL_CHANGE);
                }
            }
            pinPoint.update();

            double currentX = pinPoint.getPosX(DistanceUnit.CM);
            double currentY = pinPoint.getPosY(DistanceUnit.CM);
            double heading = pinPoint.getHeading(AngleUnit.DEGREES);

            // 算機器到目標的誤差
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double distanceToTarget = Math.hypot(errorX, errorY);

            // 計算角度誤差
            double errorHeading = angleWrap(targetHeadingDeg - heading);

            //確認誤差是否達到允許值
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
            double robotVx = targetVxGlobal * Math.cos(headingRad) - targetVyGlobal * Math.sin(headingRad);
            double robotVy = targetVxGlobal * Math.sin(headingRad) + targetVyGlobal * Math.cos(headingRad);

            // 計算四輪馬達功率

            double powerFL = robotVy + robotVx + targetOmega;
            double powerFR = robotVy - robotVx - targetOmega;
            double powerRL = robotVy - robotVx + targetOmega; // 使用 RL
            double powerRR = robotVy + robotVx - targetOmega;


            // 限制輸出功率
            mFL.setPower(clamp(powerFL, -1, 1));
            mFR.setPower(clamp(powerFR, -1, 1));
            mRL.setPower(clamp(powerRL, -1, 1)); // 更改變數名稱
            mRR.setPower(clamp(powerRR, -1, 1)); // 更改變數名稱

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
    private static boolean checkCollision() {
        // 檢查 IMU 實例是否可轉型為 BNO055IMU (常見的 IMU 實現)
        if (!(imu instanceof BNO055IMU)) {
            // 如果不是 BNO055IMU，則無法確定方法，為了不編譯錯誤，我們返回 false
            return false;
        }

        // 將 IMU 向下轉型為 BNO055IMU
        BNO055IMU bno055Imu = (BNO055IMU) imu;

        //這裡假設 getOverallAcceleration() 是可編譯的
        Acceleration currentAceel = bno055Imu.getOverallAcceleration();

        // ... (後續碰撞檢測邏輯不變)

        // 確保加速度讀數有效
        if (currentAceel == null) {
            return false;
        }

        // 第一次執行，只儲存數據，不檢測碰撞
        if (lastAceel == null) {
            lastAceel = currentAceel;
            return false;
        }

        // 計算加速度在 X 軸和 Y 軸上的絕對變化量 (即 Jerk 的分子)
        double deltaAccelX = Math.abs(currentAceel.xAccel - lastAceel.xAccel);
        double deltaAccelY = Math.abs(currentAceel.yAccel - lastAceel.yAccel);

        // 更新上一次的加速度數據
        lastAceel = currentAceel;

        // 如果任一軸的加速度變化超過閾值，視為發生猛烈碰撞
        return deltaAccelX > COLLISION_THRESHOLD_ACCEL_CHANGE || deltaAccelY > COLLISION_THRESHOLD_ACCEL_CHANGE;
    }
}

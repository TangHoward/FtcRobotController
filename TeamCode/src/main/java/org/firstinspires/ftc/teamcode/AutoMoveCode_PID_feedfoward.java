package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//修改日期 20250530
@Autonomous
// 這個我有用AI幫我把程式的排版都用得更好
public class AutoMoveCode_PID_feedfoward extends LinearOpMode {

    //目標位置
    double targetX = 100;     // cm
    double targetY = 100;     // cm
    double targetHeadingDeg = 180; // 度

    // PID
    double kP_pos = 0.03;
    double kI_pos = 0;
    double kD_pos = 0.001;

    // PID 參數 (角度)
    double kP_head = 0.015;
    double kI_head = 0;
    double kD_head = 0.0008;

    //允許誤差
    double positionTolerance = 0.5; // cm
    double headingTolerance = 1.0;  // 度

    // Motion Profile 參數
    double maxSpeed =   1;  // 最大馬達功率 (0~1)
    double maxAccel = 1; // 最大加速度 (馬達功率變化量/迴圈)

    // PID
    double integralX = 0, lastErrorX = 0;
    double integralY = 0, lastErrorY = 0;
    double integralHeading = 0, lastErrorHeading = 0;

    // Feedforward 參數
    double kV = 1.0;   // 與速度成比例，視摩擦與負載而定
    double kS = 0.05;  // 起始推力（克服靜摩擦）


    // 現在的速度 (用於加速度限制)
    double currentVx = 0;
    double currentVy = 0;

    DcMotorEx bl, fl, fr, br;
    GoBildaPinpointDriver pinPoint;

    @Override
    public void runOpMode() throws InterruptedException {
        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinPoint.setOffsets(100, 100,DistanceUnit.MM);
        pinPoint.setEncoderResolution(336.88, DistanceUnit.CM);

        bl = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        fl = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        fr = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        br = hardwareMap.get(DcMotorEx.class, "back_right_motor");

        bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
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

            // 判斷是否到達目標
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

            // Feedforward 計算（根據期望速度）
            double ffVx = kV * targetVxGlobal + kS * Math.signum(targetVxGlobal);
            double ffVy = kV * targetVyGlobal + kS * Math.signum(targetVyGlobal);

            // PID + Feedforward 合併速度
            targetVxGlobal += ffVx;
            targetVyGlobal += ffVy;


            // 簡單講就是 限速（Motion Profile - 速度限制）
            double targetSpeed = Math.hypot/*這東西就是畢氏定理去算斜邊*/(targetVxGlobal, targetVyGlobal);
            if (targetSpeed > maxSpeed) {
                double scale = maxSpeed / targetSpeed;
                targetVxGlobal *= scale;
                targetVyGlobal *= scale;
            }

            // Motion Profile - 加速度限制
            currentVx = limitAccel(currentVx, targetVxGlobal, maxAccel);
            currentVy = limitAccel(currentVy, targetVyGlobal, maxAccel);

            // 全局速度轉換成本體座標速度
            double headingRad = Math.toRadians(heading);
            double robotVx =  currentVx * Math.cos(headingRad) + currentVy * Math.sin(headingRad);
            double robotVy = -currentVx * Math.sin(headingRad) + currentVy * Math.cos(headingRad);

            // 計算四輪馬達功率
            double powerBL = robotVx + robotVy - targetOmega;
            double powerFL = robotVx - robotVy - targetOmega;
            double powerFR = robotVx + robotVy + targetOmega;
            double powerBR = robotVx - robotVy + targetOmega;

            // 限制輸出功率
            bl.setPower(clamp(powerBL, -1, 1));
            fl.setPower(clamp(powerFL, -1, 1));
            fr.setPower(clamp(powerFR, -1, 1));
            br.setPower(clamp(powerBR, -1, 1));

            telemetry.addData("Position", "X=%.2f Y=%.2f Dist=%.2f", currentX, currentY, distanceToTarget);
            telemetry.addData("Heading", "Current=%.2f Target=%.2f Error=%.2f", heading, targetHeadingDeg, errorHeading);
            telemetry.addData("Speed", "Vx=%.3f Vy=%.3f Omega=%.3f", robotVx, robotVy, targetOmega);
            telemetry.addData("MotorsSpeed","fl=%.2f bl=%.2f br=%.2f fr=%.2f", fl.getPower(), bl.getPower(), br.getPower(), fr.getPower());
            telemetry.update();
        }

        stopMotors();
    }

    // 限制加速度
    double limitAccel(double currentSpeed, double targetSpeed, double maxAccel) {
        double delta = targetSpeed - currentSpeed;
        if (delta > maxAccel) {
            delta = maxAccel;
        } else if (delta < -maxAccel) {
            delta = -maxAccel;
        }
        return currentSpeed + delta;
    }

    void stopMotors() {
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    // 解決里程計的角度問題
    double angleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // 就只是更方便寫程式
    double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}

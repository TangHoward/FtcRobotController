package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.TeleOpHeadingPD;
import org.firstinspires.ftc.teamcode.pedroPathing.otherclass.Configablestuff;

import java.lang.annotation.Target;
import java.util.function.Supplier;
@Configurable
@TeleOp(name = "操控程式",group = "RED")
public class move extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;

    private DcMotorEx intake, shooter;

    private Servo transferServo1, transferServo2;

    private TeleOpHeadingPD teleOpHeadingPD;

    private double targetHeading = 0;

    @Override
    public void init() {

        intake = hardwareMap.get(DcMotorEx.class,"intake");
        shooter = hardwareMap.get(DcMotorEx.class,"shooter");
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Configablestuff.botPose== null ? new Pose(72,24,0) : Configablestuff.botPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        teleOpHeadingPD = new TeleOpHeadingPD(Configablestuff.HEADING_KP,Configablestuff.HEADING_KD, targetHeading);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();


        double currentTimeSeconds = getRuntime();
        teleOpHeadingPD.setCoefficients(Configablestuff.HEADING_KP,Configablestuff.HEADING_KD);
        //teleOpHeadingPD.setTargetHeading(Math.atan2(144-follower.getPose().getY(),132 - follower.getPose().getX()));

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //移動的程式 且看有沒有在慢速模式 (當接近較小的射擊區域時機器會自動瞄準)
            if (!slowMode) {
                if(follower.getPose().getX() >= 48 &&
                        follower.getPose().getX() <= 96 &&
                        follower.getPose().getY() >= 0 &&
                        follower.getPose().getY() <= 48) {
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            teleOpHeadingPD.calculateTurnPower(follower.getHeading(), currentTimeSeconds),
                            false // Robot Centric
                    );
                }else
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x ,
                            false // Robot Centric
                    );

            }
            else{
                if(follower.getPose().getX() >= 48 &&
                        follower.getPose().getX() <= 96 &&
                        follower.getPose().getY() >= 0 &&
                        follower.getPose().getY() <= 48){
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y * Configablestuff.SLOW_MODE_MULTIPLIER,
                            -gamepad1.left_stick_x * Configablestuff.SLOW_MODE_MULTIPLIER,
                            teleOpHeadingPD.calculateTurnPower(follower.getHeading(), currentTimeSeconds),
                            false // Robot Centric
                    );
                }else
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y * Configablestuff.SLOW_MODE_MULTIPLIER,
                            -gamepad1.left_stick_x * Configablestuff.SLOW_MODE_MULTIPLIER,
                            -gamepad1.right_stick_x * Configablestuff.SLOW_MODE_MULTIPLIER,
                            false // Robot Centric
                    );

            }//This is how it looks with slowMode on

            intake.setPower(gamepad1.b ? 1 : 0);

                                // 每分轉速/60 * PPR(Pus Per Rotation)
            shooter.setVelocity(gamepad1.y ? (6000/60)*28 : 0);
        }

        //Automated PathFollowing
        if (gamepad2.aWasPressed() && false) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad2.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad2.leftBumperWasPressed()) {
            slowMode = !slowMode;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("slowMode", slowMode);
        telemetryM.addData("第一個人 :" +
                "\nB 是讓intake轉 A 是射球" ,
                "\n第二個人:" +
                "LB 是開關慢速模式");
    }
}

package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;
import java.util.List;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleopWithActions")

public class TeleopWithActions extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    // dashboard telemetry variables
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    ElapsedTime loopTimer = new ElapsedTime();

    Robot yoshi;

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    double maxVel = 1.0;
    boolean debugMode = false;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        //letting cam init fully and telling driver not to start
        telemetry.addData(">", "DO NOT START YET");
        telemetry.update();

        //yoshi = new Robot(hardwareMap, startPose);

        //it is done initializing ready to start!
        telemetry.addData(">", "READY TO START!");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }


    @Override
    public void loop() {
        double loop = loopTimer.milliseconds();
        loopTimer.reset();
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);


        // ******** BEGIN - base drive wheel control  *****
        // Run robot in POV mode - pushing joystick away makes robot move away regardless of rotation
        // note: The joystick goes negative when pushed forwards, so negate it
        //  Find robot's field axes in relation to joystick axes
        //double thetaRadians    = 0;  // temp use 0 for heading
        double speedControlFactor = 2.0 / 3.0;

        if (gamepad1.right_stick_button) {
            speedControlFactor = 1.0 / 3.0;
        } else if (gamepad1.left_stick_button) {
            speedControlFactor = 1.0;
        } else {
            speedControlFactor = 2.0 / 3.0;
        }

        double thetaRadians = yoshi.getHeading();  // get direction is robot facing
        double joyStick_x_axis = yoshi.expDrive(-gamepad1.left_stick_x, maxVel * speedControlFactor);
        double joyStick_y_axis = yoshi.expDrive(-gamepad1.left_stick_y, maxVel * speedControlFactor);
        double joystick_turn = yoshi.expDrive(gamepad1.right_stick_x, maxVel * speedControlFactor);
        double robotStrafe = joyStick_x_axis * Math.cos(thetaRadians) + -joyStick_y_axis * Math.sin(thetaRadians);
        double robotForward = joyStick_x_axis * Math.sin(thetaRadians) + joyStick_y_axis * Math.cos(thetaRadians);

        // calculate motor powers based on:
        //                  forward/back         turn           strafe
        double frontRight = robotForward - joystick_turn + robotStrafe;
        double backRight = robotForward - joystick_turn - robotStrafe;
        double frontLeft = robotForward + joystick_turn - robotStrafe;
        double backLeft = robotForward + joystick_turn + robotStrafe;

        double max = Math.max(Math.max(Math.abs(frontLeft), Math.abs(backLeft)), Math.max(Math.abs(frontRight), Math.abs(backRight)));
        if (max > 1) {
            frontLeft /= max;
            backLeft /= max;
            frontRight /= max;
            backRight /= max;
        }

        // directly set motor powers rather than use setDrivePowers() from RR
        yoshi.driveTrain.leftFront.setPower(frontLeft);
        yoshi.driveTrain.leftBack.setPower(backLeft);
        yoshi.driveTrain.rightFront.setPower(frontRight);
        yoshi.driveTrain.rightBack.setPower(backRight);
    }
}
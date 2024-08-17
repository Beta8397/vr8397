package org.firstinspires.ftc.teamcode.mybot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MyBotTele extends LinearOpMode {

    MyBot bot = new MyBot();

    public void runOpMode(){
        bot.init(hardwareMap);
        bot.setPose(0, 0, 0);
        waitForStart();
        while (opModeIsActive()){
            bot.updateOdometry();
            Pose pose = bot.getPose();
            Pose vel = bot.getVelocity();
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = gamepad1.left_trigger - gamepad1.right_trigger;
            bot.setDrivePower(px, py, pa);
            telemetry.addData("Pos","x %.1f y %.1f h %.1f", pose.x, pose.y,
                    Math.toDegrees(pose.h));
            telemetry.addData("Vel", "vx %.1f vy %.1f va %.1f",
                    vel.x, vel.y, Math.toDegrees(vel.h));
            telemetry.update();
        }
    }

}

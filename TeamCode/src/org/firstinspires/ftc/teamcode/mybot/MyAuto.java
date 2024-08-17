package org.firstinspires.ftc.teamcode.mybot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public abstract class MyAuto extends LinearOpMode {

    protected MyBot bot;

    public void setBot(MyBot bot){
        this.bot = bot;
    }

    public void driveTo(MotionProfile mProfile, double targetX, double targetY,
                        double targetHeadingDegrees, double tolerance){
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        Pose startPose = bot.getPose();

        while (opModeIsActive()){
            bot.updateOdometry();
            Pose pose = bot.getPose();
            VectorF fromStart = new VectorF((float)(pose.x - startPose.x), (float)(pose.y - startPose.y));
            VectorF toTarget = new VectorF((float)(targetX - pose.x), (float)(targetY - pose.y));
            double d1 = fromStart.magnitude();
            double d2 = toTarget.magnitude();

            if (d2 < tolerance){
                break;
            }

            VectorF dir = toTarget.multiplied(1.0f/(float)d2);

            double v1 = Math.sqrt(mProfile.vMin * mProfile.vMin + 2.0 * mProfile.accel * d1);
            double v2 = Math.sqrt(mProfile.vMin * mProfile.vMin + 2.0 * mProfile.accel * d2);
            double v = Math.min(v1, v2);
            v = Math.min(v, mProfile.vMax);

            double vX = v * dir.get(0);
            double vY = v * dir.get(1);

            double sin = Math.sin(pose.h);
            double cos = Math.cos(pose.h);

            double vXR = vX * sin - vY * cos;
            double vYR = vX * cos + vY * sin;

            double vA = 2.0 * AngleUnit.normalizeRadians(targetHeadingRadians - pose.h);

            bot.setDriveSpeed(vXR, vYR, vA);
        }

        bot.setDriveSpeed(0, 0, 0);
    }

    public void turnTo(double targetHeadingDegrees, double vaMaxDegrees, double coeff, double toleranceDegrees){
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        double vaMaxRadians = Math.toRadians(vaMaxDegrees);
        double toleranceRadians = Math.toRadians(toleranceDegrees);

        while (opModeIsActive()){
            bot.updateOdometry();
            Pose pose = bot.getPose();
            Pose velocity = bot.getVelocity();
            double offset = AngleUnit.normalizeRadians(targetHeadingRadians - pose.h);

            if (Math.abs(offset) < toleranceRadians && Math.abs(velocity.h) < 0.5){
                break;
            }

            double va = coeff * offset;
            if (Math.abs(va) > vaMaxRadians){
                va = vaMaxRadians * Math.signum(va);
            }

            bot.setDriveSpeed(0, 0, va);
        }

        bot.setDrivePower(0, 0, 0);
    }


}

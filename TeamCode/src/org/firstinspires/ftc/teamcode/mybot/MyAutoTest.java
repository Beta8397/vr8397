package org.firstinspires.ftc.teamcode.mybot;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.MotionProfile;

@Autonomous
public class MyAutoTest extends MyAuto {

    // Create a MyBot object; we still have to initialize it in the runOpMode method
    MyBot bot = new MyBot();
    public void runOpMode() {

        // Initialize the MyBot objec
        bot.init(hardwareMap);

        // Provide MyAuto with a reference to the MyBot object
        setBot(bot);

        // Tell the MyBot object where it is starting on the field (coordinates and orientation)
        bot.setPose(0, 0, 0);

        // Wait for the start button to be pressed
        waitForStart();

        /* Drive straight from the starting position to the specified coordinates, while maintaining the
           specified orientation.
         */
//        driveTo(new MotionProfile(10, 30, 10), 48, 0, 90, 1);

        // Turn to the specified heading
//        turnTo(180, 90, 6, 1);

        // Drive straight to new specified position
//        driveTo(new MotionProfile(10, 30, 10), 0, 48, 180, 1);

//        turnTo(0, 90, 6, 1);

        MotionProfile slow = new MotionProfile(5, 15, 5);

        MotionProfile fast = new MotionProfile(15, 50, 20);

        MotionProfile normal = new MotionProfile(10, 30, 10);

//        driveTo(slow, 0, -24, 0, 1);

//        driveTo(fast, 0, 24, 0, 1);

//        turnTo(-90, 90, 6, 1);

//        driveTo(slow, 48, 24, -90, 1);

        splineTo(normal, 0, 48, 90, 1, false);

        

    }

}

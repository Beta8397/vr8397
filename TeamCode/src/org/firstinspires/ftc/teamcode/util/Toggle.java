package org.firstinspires.ftc.teamcode.util;

public class Toggle {
    Predicate predicate;

    private boolean prevValue = false;

    public Toggle(Predicate pred){
        predicate = pred;
    }

    public boolean update(){
        boolean value = predicate.test();
        if (value && !prevValue){
            prevValue = true;
            return true;
        } else if (!value) {
            prevValue = false;
            return false;
        } else {
            return false;
        }
    }
}

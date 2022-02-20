package frc.robot.util;

public class LogDedupe {

    private static String lastMessage = null;
    private static int lastRepeats = 0;

    public static void log(String message) {

        if (lastMessage == null) {
            lastMessage = message;
            lastRepeats = 0;
            System.err.println(lastMessage);
        }
        else if (lastMessage.equals(message)) {
            lastRepeats += 1;
            if (lastRepeats % 100 == 99) {
                System.err.println(message+" ("+lastRepeats+" repeats)");
                lastRepeats = 0;
            }
        }
        else {
            lastMessage = message;
            lastRepeats = 0;
            System.err.println(message);
       }
    }
    
}

package frc.robot.util;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;

public class Cloner implements Cloneable {

    public static <T extends Serializable> T copy(T target) {
        try {

            ByteArrayOutputStream bytesOut = new ByteArrayOutputStream();
            ObjectOutputStream out = new ObjectOutputStream(bytesOut);
            out.writeObject(target);
            out.close();

            ByteArrayInputStream bytesIn = new ByteArrayInputStream(bytesOut.toByteArray());
            ObjectInputStream in = new ObjectInputStream(bytesIn);
            T copy = (T) in.readObject();
            in.close();

            return copy;
        }
        catch(Exception e) {
            throw new RuntimeException(e);
        }
    }
}

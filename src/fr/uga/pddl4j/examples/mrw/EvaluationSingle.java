package fr.uga.pddl4j.examples.mrw;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class EvaluationSingle {
    public static void main(String[] args) {

        String path = "p20.pddl";

        
        try (BufferedWriter writer = new BufferedWriter(new FileWriter("gripperTime.txt", true))) {
            writer.write(/*/"\n" + path+*/";");
        } catch (IOException e) {
            e.printStackTrace();
        }
        try (BufferedWriter writer = new BufferedWriter(new FileWriter("gripperLength.txt", true))) {
            writer.write(/*"\n" + path +*/ ";");
        } catch (IOException e) {
            e.printStackTrace();
        }
        MyPlanner.main(new String[] { ".\\gripper\\domain.pddl", ".\\gripper\\" + path, "gripperTime.txt",
                "gripperLength.txt" });

    }
}

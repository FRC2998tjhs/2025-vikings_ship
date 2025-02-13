package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.dyn4j.geometry.Vector3;

public class Field {
    public static final Field workshop = new Field()
        .withTag(14, new Vector3(2.1336, -0.12065, 0.8763))
        .withTag(15, new Vector3(-0.1778, -0.762, 0.8001))
        .withTag(16, new Vector3(2.2733, 3.1369, 0.78105))
        .withTag(17, new Vector3(2.9337, 1.5748, 0.7112))
        .withTag(12, new Vector3(0.3937, 2.2225, 1.0414));

    // Will's Notes for AprilTag positions:
    // 1: Red Right
    // 2: A Left
    // 3: Red processor
    // 4: RB shoot
    // 5: RR Shoot
    // [6, 11]: Reef
    // 12: blue left drop
    // 13: blue right
    // 14: BB shoot
    // 15: BR shoot
    // 16: B processor
    // [17, 22]: Reef

    private Map<Integer, Vector3> tags = new HashMap<Integer, Vector3>();

    Field withTag(int id, Vector3 pos) {
        this.tags.put(id, pos);
        return this;
    }

    public Vector3 tagPosition(int tagId) {
        return this.tags.get(tagId);
    }

}

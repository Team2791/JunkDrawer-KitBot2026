package frc.robot.auto;

import frc.robot.auto.generated.ChoreoTraj;
import java.util.EnumMap;
import java.util.Map;
import java.util.Set;

/**
 * A graph-based representation of autonomous navigation paths for the robot.
 *
 * <p>This class builds a transition map that connects autonomous nodes (positions/states)
 * through Choreo trajectories. It automatically parses trajectory names to determine
 * the source and destination nodes for each path.
 *
 * <p>The class expects trajectory names to follow the format "from_to" where "from" and "to"
 * are valid AutoNode enum values (lowercased). Trajectories with names starting with "seq_" or not
 * containing exactly one underscore are ignored.
 */
public class AutoGraph {

    final Map<AutoNode, Map<AutoNode, ChoreoTraj>> TRANSITIONS = calculate();

    static Map<AutoNode, Map<AutoNode, ChoreoTraj>> calculate() {
        Map<AutoNode, Map<AutoNode, ChoreoTraj>> out = new EnumMap<>(
            AutoNode.class
        );

        for (ChoreoTraj traj : ChoreoTraj.ALL_TRAJECTORIES.values()) {
            String name = traj.name();
            if (name.startsWith("seq_")) continue;
            if (!name.contains("_")) continue;

            String[] parts = name.split("_");
            if (parts.length != 2) continue;

            AutoNode from;
            AutoNode to;

            try {
                from = AutoNode.valueOf(parts[0].toUpperCase());
                to = AutoNode.valueOf(parts[1].toUpperCase());
            } catch (IllegalArgumentException e) {
                continue;
            }

            out.putIfAbsent(from, new EnumMap<>(AutoNode.class));
            out.get(from).put(to, traj);
        }

        for (AutoNode from : out.keySet()) {
            System.out.print(from.toString() + " -> ");
            Set<AutoNode> tos = out.get(from).keySet();
            System.out.println(tos);
        }

        return out;
    }

    public AutoGraph() {}

    /**
     * Finds the Choreo trajectory that transitions from one AutoNode to another.
     * @param from The starting AutoNode
     * @param to The destination AutoNode
     * @return The ChoreoTraj that connects from and to, or null if no such trajectory exists
     */
    public ChoreoTraj transition(AutoNode from, AutoNode to) {
        if (!TRANSITIONS.containsKey(from)) return null;
        return TRANSITIONS.get(from).get(to);
    }

    /**
     * Checks if a valid trajectory transition exists between two AutoNodes.
     * @param from The starting AutoNode
     * @param to The destination AutoNode
     * @return True if a trajectory exists that transitions from "from" to "to", false otherwise
     */
    public boolean hasTransition(AutoNode from, AutoNode to) {
        if (!TRANSITIONS.containsKey(from)) return false;
        return TRANSITIONS.get(from).containsKey(to);
    }

    /**
     * Returns a set of AutoNodes that can be transitioned to from the given AutoNode.
     * @param from The starting AutoNode
     * @return A set of AutoNodes that have a valid trajectory transition from the given "from" node. If no transitions exist, returns an empty set.
     */
    public Set<AutoNode> transitionsFrom(AutoNode from) {
        if (!TRANSITIONS.containsKey(from)) return Set.of();
        return TRANSITIONS.get(from).keySet();
    }
}

package fr.uga.pddl4j.examples.mrw;

import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.operator.Condition;
import fr.uga.pddl4j.util.BitVector;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import picocli.CommandLine;

/**
 * The class is an example. It shows how to create a PRW search planner
 * able to
 * solve an ADL problem by choosing the heuristic to used and its weight.
 *
 * @author D. Pellier
 * @version 4.0 - 30.11.2021
 */
@CommandLine.Command(name = "ASP", version = "ASP 1.0", description = "Solves a specified planning problem using A* search strategy.", sortOptions = false, mixinStandardHelpOptions = true, headerHeading = "Usage:%n", synopsisHeading = "%n", descriptionHeading = "%nDescription:%n%n", parameterListHeading = "%nParameters:%n", optionListHeading = "%nOptions:%n")

public class MyPlannerImprove extends AbstractPlanner {

    private double heuristicWeight;
    private StateHeuristic.Name heuristic;
    /**
     * The class logger.
     */
    private static final Logger LOGGER = LogManager.getLogger(MyPlannerImprove.class.getName());

    /**
     * Instantiates the planning problem from a parsed problem.
     *
     * @param problem the problem to instantiate.
     * @return the instantiated planning problem or null if the problem cannot be
     *         instantiated.
     */
    @Override
    public Problem instantiate(DefaultParsedProblem problem) {
        final Problem pb = new DefaultProblem(problem);
        pb.instantiate();
        return pb;
    }

    /**
     * Évalue l'état actuel par rapport à l'objectif et retourne un score
     * heuristique.
     * 
     * @param currentState L'état à évaluer.
     * @param goal         L'objectif à atteindre.
     * @return Un score heuristique (plus il est bas, plus l'état est proche de
     *         l'objectif).
     */
    private double evaluateHeuristic(State currentState, Condition goal) {
        double heuristicValue = 0.0;

        // Vérifie les fluents positifs de l'objectif
        BitVector goalPositiveFluents = goal.getPositiveFluents();
        for (int i = 0; i < goalPositiveFluents.size(); i++) {
            if (goalPositiveFluents.get(i) && !currentState.get(i)) {
                heuristicValue += 1.0; // Incrémente le score pour chaque fluent positif non satisfait
            }
        }

        // Vérifie les fluents négatifs de l'objectif
        BitVector goalNegativeFluents = goal.getNegativeFluents();
        for (int i = 0; i < goalNegativeFluents.size(); i++) {
            if (goalNegativeFluents.get(i) && currentState.get(i)) {
                heuristicValue += 1.0; // Incrémente le score si un fluent négatif est présent dans l'état actuel
            }
        }

        // Retourne le score heuristique
        return heuristicValue;
    }

    /**
     * Search a solution plan to a specified domain and problem using PWR.
     *
     * @param problem the problem to solve.
     * @return the plan found or null if no plan was found.
     */
    @Override
    public Plan solve(final Problem problem) {
        Plan plan = new SequentialPlan();
        List<Action> bestActionSequence = new ArrayList<>();
        State initialState = new State(problem.getInitialState());
        Condition goal = problem.getGoal();
        int numWalks = 4000; // Nombre maximum de marches aléatoires
        int maxLengthWalk = 30; // Longueur maximale d'une marche aléatoire
        Random random = new Random();
        double bestHeuristicValue = Double.MAX_VALUE;

        // Seuils pour MDA et MHA
        int deadEndCount = 0;
        int totalWalks = 0;
        int totalBranchingFactor = 0; // Somme des facteurs de branchement pour calcul de la moyenne
        boolean useMDA = false;
        boolean useMHA = false;

        // Structures pour MDA et MHA
        Map<Action, Integer> successCount = new HashMap<>();
        Map<Action, Integer> failureCount = new HashMap<>();
        Map<Action, Integer> helpfulActionCount = new HashMap<>();

        for (Action action : problem.getActions()) {
            successCount.put(action, 0);
            failureCount.put(action, 0);
            helpfulActionCount.put(action, 0);
        }

        LOGGER.info("* Starting Pure Random Walk search with threshold-based MDA and MHA\n");

        for (int i = 0; i < numWalks; i++) {
            State currentState = new State(initialState);
            List<Action> currentActionSequence = new ArrayList<>();
            boolean walkSucceeded = false;
            boolean walkHitDeadEnd = false;

            for (int j = 0; j < maxLengthWalk; j++) {
                List<Action> actions = problem.getActions();
                List<Action> applicableActions = new ArrayList<>();

                for (Action action : actions) {
                    if (action.isApplicable(currentState)) {
                        applicableActions.add(action);
                    }
                }

                // Mettre à jour le facteur de branchement total (average branching factor)
                totalBranchingFactor += applicableActions.size();

                if (applicableActions.isEmpty()) {
                    walkHitDeadEnd = true;
                    deadEndCount++;
                    break;
                }

                Action selectedAction;

                // Appliquer MHA ou MDA selon les seuils
                if (useMHA) {
                    List<Action> preferredActions = applicableActions.stream()
                            .filter(action -> helpfulActionCount.get(action) > 0)
                            .collect(Collectors.toList());
                    selectedAction = preferredActions.isEmpty()
                            ? applicableActions.get(random.nextInt(applicableActions.size()))
                            : preferredActions.get(random.nextInt(preferredActions.size()));
                } else if (useMDA) {
                    selectedAction = selectActionWithMDA(applicableActions, failureCount, random);
                } else {
                    selectedAction = applicableActions.get(random.nextInt(applicableActions.size())); // Pure Random
                                                                                                      // Walk
                }

                currentState.apply(selectedAction.getUnconditionalEffect());
                currentActionSequence.add(selectedAction);

                if (currentState.satisfy(goal)) {
                    walkSucceeded = true;
                    successCount.put(selectedAction, successCount.get(selectedAction) + 1);
                    helpfulActionCount.put(selectedAction, helpfulActionCount.get(selectedAction) + 1);
                    break;
                } else {
                    failureCount.put(selectedAction, failureCount.get(selectedAction) + 1);
                }
            }

            // Vérifier si MDA ou MHA doivent être activés
            totalWalks++;
            if (totalWalks % 100 == 0) { // toutes les 100 marches
                if ((double) deadEndCount / totalWalks >= 0.5) {
                    useMDA = true;
                    LOGGER.info("Activating MDA due to high dead-end rate.\n");
                }

                // Calcul du facteur de branchement moyen et activation de MHA si nécessaire
                int averageBranchingFactor = totalBranchingFactor / totalWalks;
                if (averageBranchingFactor > 1000) {
                    useMHA = true;
                    LOGGER.info("Activating MHA due to high branching factor.\n");
                }
            }

            if (walkSucceeded) {
                double heuristicValue = evaluateHeuristic(currentState, goal);
                if (heuristicValue < bestHeuristicValue) {
                    bestHeuristicValue = heuristicValue;
                    bestActionSequence = new ArrayList<>(currentActionSequence);
                }
            }
        }

        if (!bestActionSequence.isEmpty()) {
            LOGGER.info("* Enhanced Random Walk search found a solution\n");
            for (int i = 0; i < bestActionSequence.size(); i++) {
                plan.add(i, bestActionSequence.get(i));
            }
            return plan;
        } else {
            LOGGER.info("* Enhanced Random Walk search failed to find a solution within the walk limit\n");
            return null;
        }
    }

    /**
     * Sélectionne une action en utilisant MDA seulement si l'activation est
     * autorisée.
     */
    private Action selectActionWithMDA(List<Action> applicableActions, Map<Action, Integer> failureCount,
            Random random) {
        int minFailures = Integer.MAX_VALUE;
        List<Action> bestActions = new ArrayList<>();

        for (Action action : applicableActions) {
            int failures = failureCount.getOrDefault(action, 0);
            if (failures < minFailures) {
                minFailures = failures;
                bestActions.clear();
                bestActions.add(action);
            } else if (failures == minFailures) {
                bestActions.add(action);
            }
        }

        return bestActions.get(random.nextInt(bestActions.size()));
    }

    /**
     * Returns if a specified problem is supported by the planner. Just ADL problem
     * can be solved by this planner.
     *
     * @param problem the problem to test.
     * @return <code>true</code> if the problem is supported <code>false</code>
     *         otherwise.
     */
    @Override
    public boolean isSupported(Problem problem) {
        return !problem.getRequirements().contains(RequireKey.ACTION_COSTS)
                && !problem.getRequirements().contains(RequireKey.CONSTRAINTS)
                && !problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
                && !problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
                && !problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
                && !problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
                && !problem.getRequirements().contains(RequireKey.FLUENTS)
                && !problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
                && !problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
                && !problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
                && !problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
                && !problem.getRequirements().contains(RequireKey.PREFERENCES)
                && !problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
                && !problem.getRequirements().contains(RequireKey.HIERARCHY);
    }

    /**
     * The main method of the <code>ASP</code> planner.
     *
     * @param args the arguments of the command line.
     */
    public static void main(String[] args) {
        try {
            final MyPlannerImprove planner = new MyPlannerImprove();
            CommandLine cmd = new CommandLine(planner);
            cmd.execute(args);
        } catch (IllegalArgumentException e) {
            LOGGER.fatal(e.getMessage());
        }
    }

    /**
     * Returns the name of the heuristic used by the planner to solve a planning
     * problem.
     *
     * @return the name of the heuristic used by the planner to solve a planning
     *         problem.
     */
    public final StateHeuristic.Name getHeuristic() {
        return this.heuristic;
    }

    /**
     * Returns the weight of the heuristic.
     *
     * @return the weight of the heuristic.
     */
    public final double getHeuristicWeight() {
        return this.heuristicWeight;
    }

    /**
     * Sets the weight of the heuristic.
     *
     * @param weight the weight of the heuristic. The weight must be greater than 0.
     * @throws IllegalArgumentException if the weight is strictly less than 0.
     */
    @CommandLine.Option(names = { "-w",
            "--weight" }, defaultValue = "1.0", paramLabel = "<weight>", description = "Set the weight of the heuristic (preset 1.0).")
    public void setHeuristicWeight(final double weight) {
        if (weight <= 0) {
            throw new IllegalArgumentException("Weight <= 0");
        }
        this.heuristicWeight = weight;
    }

    /**
     * Set the name of heuristic used by the planner to the solve a planning
     * problem.
     *
     * @param heuristic the name of the heuristic.
     */
    @CommandLine.Option(names = { "-e",
            "--heuristic" }, defaultValue = "FAST_FORWARD", description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, "
                    + "MAX, FAST_FORWARD SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public void setHeuristic(StateHeuristic.Name heuristic) {
        this.heuristic = heuristic;
    }
}


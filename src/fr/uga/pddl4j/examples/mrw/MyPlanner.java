package fr.uga.pddl4j.examples.mrw;

import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.SearchStrategy;
import fr.uga.pddl4j.planners.statespace.search.StateSpaceSearch;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.operator.Condition;
import fr.uga.pddl4j.util.BitVector;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

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

public class MyPlanner extends AbstractPlanner {

    private double heuristicWeight;
    private StateHeuristic.Name heuristic;
    private String timeFile;
    private String lengthFile;

    /**
     * The class logger.
     */
    private static final Logger LOGGER = LogManager.getLogger(MyPlanner.class.getName());

    private void setTimeFile(String resultFile) {
        this.timeFile = resultFile;
    }

    private void setLengthFile(String resultFile) {
        this.lengthFile = resultFile;
    }

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
        // Créez une instance de plan
        Plan plan = new SequentialPlan(); // Remplacez par votre implémentation de l'interface Plan
        List<Action> bestActionSequence = new ArrayList<>();
        State initialState = new State(problem.getInitialState());
        Condition goal = problem.getGoal();
        int numWalks = 2000; // Nombre maximum de marches aléatoires
        int maxLengthWalk = 10; // Longueur maximale d'une marche aléatoire
        Random random = new Random();
        double bestHeuristicValue = Double.MAX_VALUE;

        LOGGER.info("* Starting Pure Random Walk search (MRW)\n");

        for (int i = 0; i < numWalks; i++) {
            State currentState = new State(initialState);
            List<Action> currentActionSequence = new ArrayList<>();

            // Effectuer une marche aléatoire
            for (int j = 0; j < maxLengthWalk; j++) {
                List<Action> actions = problem.getActions(); // Obtenir toutes les actions disponibles
                List<Action> applicableActions = new ArrayList<>();

                // Filtrer les actions applicables dans l'état actuel
                for (Action action : actions) {
                    if (action.isApplicable(currentState)) {
                        applicableActions.add(action);
                    }
                }

                // Arrêter la marche si aucune action n'est applicable
                if (applicableActions.isEmpty()) {
                    break;
                }

                // Sélection d'une action aléatoire parmi les actions applicables
                Action randomAction = applicableActions.get(random.nextInt(applicableActions.size()));
                currentState.apply(randomAction.getUnconditionalEffect()); // Appliquer l'effet de l'action
                currentActionSequence.add(randomAction);

                // Si l'objectif est atteint pendant la marche, arrêter et retourner le plan
                if (currentState.satisfy(goal)) {
                    LOGGER.info("* Solution found during a random walk\n");
                    for (int k = 0; k < currentActionSequence.size(); k++) {
                        plan.add(k, currentActionSequence.get(k));
                    }

                    if(timeFile != null && lengthFile != null) {
                        StringBuilder rawTime = new StringBuilder();
                        StringBuilder rawLength = new StringBuilder();

                        String timeString = String.valueOf(this.getStatistics().getTimeToSearch()
                                + this.getStatistics().getTimeToEncode() + this.getStatistics().getTimeToParse());

                        rawTime.append(";" + timeString);
                        rawLength.append(";" + plan.actions().size());

                        // ecrire les résultats dans le fichier
                        try (BufferedWriter writer = new BufferedWriter(new FileWriter(this.timeFile, true))) {
                            writer.write(rawTime.toString());
                        } catch (IOException e) {
                            e.printStackTrace();
                        }

                        try (BufferedWriter writer = new BufferedWriter(new FileWriter(this.lengthFile, true))) {
                            writer.write(rawLength.toString());
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                    }

                    return plan;
                }
            }

            // Évaluation de l'état final de la marche
            double heuristicValue = evaluateHeuristic(currentState, goal); // Fonction heuristique

            // Si l'état final a une meilleure valeur heuristique, le sauvegarder
            if (heuristicValue < bestHeuristicValue) {
                bestHeuristicValue = heuristicValue;
                bestActionSequence = new ArrayList<>(currentActionSequence);
            }
        }

        // Vérifier si une séquence d'actions valide a été trouvée
        if (!bestActionSequence.isEmpty()) {
            LOGGER.info("* Pure Random Walk search found a solution with the best heuristic value\n");
            for (int i = 0; i < bestActionSequence.size(); i++) {
                plan.add(i, bestActionSequence.get(i));
            }

            if(timeFile != null && lengthFile != null) {
                StringBuilder rawTime = new StringBuilder();
                StringBuilder rawLength = new StringBuilder();

                String timeString = String.valueOf(this.getStatistics().getTimeToSearch()
                        + this.getStatistics().getTimeToEncode() + this.getStatistics().getTimeToParse());

                rawTime.append(";" + timeString);
                rawLength.append(";" + plan.actions().size());

                // ecrire les résultats dans le fichier
                try (BufferedWriter writer = new BufferedWriter(new FileWriter(this.timeFile, true))) {
                    writer.write(rawTime.toString());
                } catch (IOException e) {
                    e.printStackTrace();
                }

                try (BufferedWriter writer = new BufferedWriter(new FileWriter(this.lengthFile, true))) {
                    writer.write(rawLength.toString());
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            return plan;
        } else {
            if(timeFile !=null && lengthFile != null) {
                try (BufferedWriter writer = new BufferedWriter(new FileWriter(this.timeFile, true))) {
                    writer.write(";");
                } catch (IOException e) {
                    e.printStackTrace();
                }
                try (BufferedWriter writer = new BufferedWriter(new FileWriter(this.lengthFile, true))) {
                    writer.write(";");
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            LOGGER.info("* Pure Random Walk search failed to find a solution within the walk limit\n");
            return null;
        }
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
            final MyPlanner planner = new MyPlanner();
            if(args.length == 4) {
                planner.setTimeFile(args[2]);
                planner.setLengthFile(args[3]);
            }
            planner.setTimeout(900);
            CommandLine cmd = new CommandLine(planner);
            cmd.execute(new String[] { args[0], args[1] });
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

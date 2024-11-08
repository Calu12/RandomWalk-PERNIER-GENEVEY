Binome : Alexandre PERNIER & Coemgen GENEVEY

<h1>Installation</h1>

Java, Git et Python doivent être installé sur la machine.

D'abord on ouvre une invite de commande dans le répertoire ou on soihaite installer le projet puis un execute la commande suivante :
```
git clone https://github.com/Calu12/RandomWalk-PERNIER-GENEVEY.git
 ```

Puis placez-vous dans le projet avec la commande
```
cd RandomWalk-PERNIER-GENEVEY
 ```

Dans votre explorateur de fichier, vérifier que le dossier "classes\fr\uga\pddl4j\examples\mrw" dans "RandomWalk-PERNIER-GENEVEY" contient bien les fichiers suivant : 
-EvaluationSingle.class
-HSP.class
-MyPlanner.class
-MyPlannerImprove.class
-ScriptEvaluation

Si certains fichier manque vous pouvez les générer avec les commandes suivantes (à exécuter dans "RandomWalk-PERNIER-GENEVEY") :
```
javac -d classes -cp lib/pddl4j-4.0.0.jar;classes src/fr/uga/pddl4j/examples/mrw/EvaluationSingle.java
 ```
```
javac -d classes -cp lib/pddl4j-4.0.0.jar;classes src/fr/uga/pddl4j/examples/mrw/HSP.java
 ```
```
javac -d classes -cp lib/pddl4j-4.0.0.jar;classes src/fr/uga/pddl4j/examples/mrw/MyPlanner.java
 ```
```
javac -d classes -cp lib/pddl4j-4.0.0.jar;classes src/fr/uga/pddl4j/examples/mrw/MyPlannerImprove.java
 ```
```
javac -d classes -cp lib/pddl4j-4.0.0.jar;classes src/fr/uga/pddl4j/examples/mrw/ScriptEvaluation.java
 ```

Pour l'affichage des graphiques, il est nécessaire d'avoir la bibliothèque python matplotlib, pour l'installer il faut exécuter la commande suivante dans un invité de commande : 
```
pip install matplotlib
 ```


<h1>Graphiques</h1>

Dans le fichier "graphs.ipynb" se trouve des graphes de comparaison entre les performances des planners HSP et Random Walk.
C'est un NoteBook python qui peut être ouvert et exécuté dans l'éditeur VSCode avec les extensions : Jupyter, Jupyter Cell Tags, Jupyter Keymap et Jupyter Slide Show.

<h1>Exécuter les classes</h1>

Les commandes ci-dessous doivent être exécutées dans "RandomWalk-PERNIER-GENEVEY" et les éléments "<domain>" et <probleme> doivent être remplacés par les chemain d'accès aux fichier de domain et de problème ".pddl" souhaités.

Pour exécuter le planner HSP : 
```
java -cp classes;lib/pddl4j-4.0.0.jar fr.uga.pddl4j.examples.mrw.HSP <domain> <probleme>
 ```

Pour exécuter le planner Random Walk (L'algorithme Monte-Carlo Pure Random Walk à été implémenter directement dans la méthode solve()): 
```
java -cp classes;lib/pddl4j-4.0.0.jar fr.uga.pddl4j.examples.mrw.MyPlanner <domain> <probleme>
 ```

Pour exécuter le planner Random Walk Amélioré (Avec l'ajout des techniques Monte-Carlo Deadlock Avoidance et Monte-Carlo with Helpful Actions vue dans l'article à l'alogorithme) : 
```
java -cp classes;lib/pddl4j-4.0.0.jar fr.uga.pddl4j.examples.mrw.MyPlannerImprove <domain> <probleme>
 ```

<strong>Je ne recommande pas d'utiliser les 2 commandes suivantes dans la mesure où la première peut prendre plusieurs heures d'exécution et où les deux vont modifier les fichiers text de données servants à afficher les graphiques dans le notebook "graphs.ipynb" !</strong>


Pour exécuter les planners HSP et Random Walk pour touts les problèmes des dossiers "gripper", "blocks", "logistics" et "depots" et ajoute les résultats de temps et de longueur de la solution dans les documents text à la base du projet :
```
java -cp classes;lib/pddl4j-4.0.0.jar fr.uga.pddl4j.examples.mrw.ScriptEvaluation
 ```

Pour exécuter les planners HSP et Random Walk pour un problème et ajoute les résultats de temps et de longueur de la solution dans les documents text à la base du projet :
```
java -cp classes;lib/pddl4j-4.0.0.jar fr.uga.pddl4j.examples.mrw.EvaluationSingle
 ```
Remarque : On peut changer le nom du fichier de problème et de son dossier dans le fichier "src\fr\uga\pddl4j\examples\mrw\EvaluationSingle.java". Il faut ensuite exécuter :
```
javac -d classes -cp lib/pddl4j-4.0.0.jar;classes src/fr/uga/pddl4j/examples/mrw/EvaluationSingle.java
 ```
Pour recompiler la classe avant de l'exécuter.

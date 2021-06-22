# Error Categories

## AutowareError

Unexpected behavior of Autoware was detected.

The most typical cause of this error is the corruption of some nodes during Autoware's launch.
This anomaly is empirically known to be less likely to occur the higher the performance of the computer running the simulator, but the detailed cause is unknown.
It may be remedied by restarting the simulator.

## InternalError

An unexpected anomaly has occurred inside the interpreter/simulator.

This error is caused by a bug in the interpreter/simulator implementation.Due to errors in error handling, there are also cases where a faulty scenario description appears as an error in this category, but there is generally no possible action on the user side.

Please report the error to the developer with the executed scenario and map (if possible).

## SemanticError

A semantic anomaly was detected during the execution of the scenario.

When this error occurs, the scenario you gave the simulator is syntactically exactly correct.
However, there is a problem with its content.

This error is similar to SimulationError, but among other things, it is categorized as when there is a strong suspicion that there is a problem with the content of the scenario.

## SimulationError

An internal problem was encountered during the execution of the simulation.

A typical error is caused by a query for the state of a non-existent entity. This is the case, for example, when TeleportAction, which needs to be called at least once for all entities, is not called.

This error is similar to SemanticError, but it is categorized as, among other things, when there is a strong suspicion that the problem is with the simulator.

## SyntaxError

The given scenario is syntactically incorrect.

This error occurs when the scenario you gave is correct as OpenSCENARIO, but uses a feature that our implementation does not yet support.

Communicating the need for the feature to the developers may help to raise the priority of the implementation.
Alternatively, if you can implement the feature yourself, please consider submitting a pull request to our interpreter/simulator.

# Tier IV Scenario Runner - OpenSCENARIO Implementation for Autoware

## Overview

Scenario Runner is an OpenSCENARIO (.xosc) interpreter to run an autonomous driving simulation.

OpenSCENARIO is an **open file format** for the description of dynamic contents in driving simulation applications provided by ASAM e.V.

Tier IV plans to use OpenSCENARIO for Autoware's Continuous Integration (CI), and is extending it for CI as far as the standard allows (see [Extensions](#Extensions) for details).

## Installation
[//]: # (TODO : remove .Architecture Proposal)
Place this repository in the ROS workspace with Autoware and build it in [the way Autoware specifies](https://github.com/tier4/AutowareArchitectureProposal#autoware-setup).

## Usage

The following commands can be used to start all the ROS nodes needed to run the scenario.

```
$ roslaunch openscenario_interpreter openscenario_interpreter.launch
```

You need to pass the following arguments to the above commands, depending on the scenario you want to run.

#### `map_path`

Specify the path to **the directory** where the lanelet2 file (.osm) is located.
Note that Autoware uses lanelet2 and not OpenDRIVE.

#### `scenario`

Specify the path to the OpenSCENARIO file (.xosc) you want to run.

example:
``` bash
$ roslaunch openscenario_interpreter openscenario_interpreter.launch map_path:=/path/to/map/directory scenario:=/path/to/scenario.xosc
```

## Errors

Scenario Runner or any of its dependent nodes may fail during startup or execution.
It may not be a problem of Scenario Runner, since multiple ROS nodes are started at the same time.

But if the problem is caused by the Scenario Runner, it should print out the cause of the error by red letters.

Since there is a general classification of errors, you can deal with them according to the following.

### Syntax Error

This error is caused by a syntax error in OpenSCENARIO.
You have to fix the OpenSCENARIO (.xosc) file that you passed to Scenario Runner.

Here are some common mistakes:
- Required elements are not present where they should be
- Mistyping the element name.

Note that if an unknown element is written, the Scenario Runner does not treat it as an error, but simply ignores it.

### Semantic Error

An error that appears when the syntax of OpenSCENARIO is correct, but the scenario is not viable.

Here are some common mistakes:
- Instructing a lane change for an entity that is not on the lane

### Implementation Fault

An error that appears when using a feature of OpenSCENARIO that is not yet implemented at that time.
It's not your fault.

There is no quick fix.

### Simulation Runtime Error

Other miscellaneous errors.

## Extensions

### UserDefinedAction

#### exitSuccess

Force termination of the scenario as successful.
The exit status is notified in an operating system specific way.

example:

``` XML
<UserDefinedAction>
  <CustomCommandAction type="exitSuccess"/>
</UserDefinedAction>
```

#### exitFailure

Force termination of the scenario as failed.
The exit status is notified in an operating system specific way.

example:

``` XML
<UserDefinedAction>
  <CustomCommandAction type="exitFailure"/>
</UserDefinedAction>
```

## Supported/Implemented Elements

Scenario Runner does not fully implement OpenSCENARIO at this time.
Checked features in the following list are already implemented.

### Action

- GlobalAction
	- [ ] EnvironmentAction
	- [ ] EntityAction
	- [ ] ParameterAction
	- [ ] InfrastructureAction
	- [ ] TrafficAction
- UserDefinedAction
	- [x] CustomCommandAction
- PrivateAction
	- LongitudinalAction
		- [x] SpeedAction
		- [ ] LongitudinalDistanceAction
	- LateralAction
		- [x] LaneChangeAction
		- [ ] LaneOffsetAction
		- [ ] LateralDistanceAction
	- [ ] VisibilityAction
	- [ ] SynchronizeAction
	- [ ] ActivateControllerAction
	- ControllerAction
		- [ ] AssignControllerAction
		- [ ] OverrideControllerAction
	- [x] TeleportAction
	- RoutingAction
		- [ ] AssignRouteAction
		- [ ] FollowTrajectoryAction
		- [x] AcquirePositionAction

### Condition

- ByEntityCondition
	- EntityCondition
		- [ ] EndOfRoadCondition
		- [ ] CollisionCondition
		- [ ] OffroadCondition
		- [x] TimeHeadwayCondition
		- [x] AccelerationCondition
		- [ ] StandStillCondition
		- [x] SpeedCondition
		- [ ] RelativeSpeedCondition
		- [ ] TraveledDistanceCondition
		- [x] ReachPositionCondition
		- [ ] DistanceCondition
		- [ ] RelativeDistanceCondition
- ByValueCondition
	- [ ] ParameterCondition
	- [ ] TimeOfDayCondition
	- [x] SimulationTimeCondition
	- [x] StoryboardElementStateCondition
	- [ ] UserDefinedValueCondition
	- [ ] TrafficSignalCondition
	- [ ] TrafficSignalControllerCondition

### Syntax

- [x] OpenSCENARIO
	- [x] FileHeader
	- [x] ParameterDeclarations
	- [ ] CatalogLocations
	- [x] RoadNetwork
		- [x] LogicFile
		- [x] SceneGraphFile
		- [ ] TrafficSignalController
	- [x] Entities
	- [x] Storyboard
		- [x] Init
			- [x] InitActions
				- [x] GlobalAction
				- [x] UserDefinedAction
				- [x] Private
					- [x] PrivateAction
		- [x] Story
			- [x] ParameterDeclarations
			- [x] Act
				- [x] ManeuverGroup
					- [x] Actors
						- [x] EntityRef
					- [ ] CatalogReference
					- [x] Maneuver
						- [x] ParameterDeclarations
						- [x] Event
							- [x] Action
								- [x] GlobalAction
								- [x] UserDefinedAction
								- [x] PrivateAction
							- [x] StartTrigger
				- [x] StartTrigger
				- [x] StopTrigger
		- [x] StopTrigger
	- [x] Catalog

---

- [x] FileHeader
	- [x] revMajor
	- [x] revMinor
	- [x] date
	- [x] description
	- [x] author

---

- [x] ParameterDeclaration
	- [x] name
	- [x] parameterType
		- [x] integer
		- [x] double
		- [x] string
		- [x] unsignedInt
		- [x] unsignedShort
		- [x] boolean
		- [x] dateTime
	- [x] value


---

- [ ] CatalogLocations
	- [ ] VehicleCatalog
	- [ ] ControllerCatalog
	- [ ] PedestrianCatalog
	- [ ] MiscObjectCatalog
	- [ ] EnvironmentCatalog
	- [ ] ManeuverCatalog
	- [ ] TrajectoryCatalog
	- [ ] RouteCatalog

---

- [x] RoadNetwork
	- [x] LogicFile
	- [x] SceneGraphFile
	- [ ] TrafficSignalController
		- [ ] name
		- [ ] delay
		- [ ] reference
		- [ ] Phase
			- [ ] name
			- [ ] duration
			- [ ] TrafficSignalState
				- [ ] trafficSignalId
				- [ ] state

---

- [x] Entities
	- [x] ScenarioObject
		- [x] name
		- [x] EntityObject
			- [ ] CatalogReference
			- [x] Vehicle
			- [x] Pedestrian
			- [ ] MiscObject
		- [ ] ObjectController
	- [ ] EntitySelection

---

- [x] Vehicle
	- [x] name
	- [x] vehicleCategory
		- [x] car
		- [ ] van
		- [ ] truck
		- [ ] trailer
		- [ ] semitrailer
		- [ ] bus
		- [ ] motorbike
		- [ ] bicycle
		- [ ] train
		- [ ] tram
	- [x] ParameterDeclarations
	- [x] BoundingBox
		- [x] ...
	- [x] Performance
		- [x] ...
	- [x] Axles
		- [x] ...
	- [ ] Properties

---

- [x] Pedestrian
	- [x] name
	- [x] model
	- [x] mass
	- [x] pedestrianCategory
		- [x] pedestrian
		- [ ] wheelchair
		- [ ] animal
	- [x] ParameterDeclarations
		- [x] ...
	- [x] BoundingBox
		- [x] ...
	- [ ] Properties

## References


- [ASAM OpenSCENARIO: User Guide](https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html)
- [OpenSCENARIO 1.0.0 XSD documentation](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/index.html)


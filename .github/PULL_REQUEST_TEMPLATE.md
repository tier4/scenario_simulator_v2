# Description

> Please check examples and comment out this sentence. Minimal example is [here](pull_request_samples/example_simple.md) and detailed example is [here](pull_request_samples/example_detail.md)

## Abstract

> [Required] This section is required, keep it short, clear and to the point.  
> Delete this sentence and explain this pull request shortly.  

## Background

> [Optional] If there is no background information that needs explanation (e.g., just a typo correction, etc.), you can skip this section.  
> Delete this sentence and explain the circumstances that led to this pull request being sent.  

## Details

> [Optional] If there are only differences whose effects are so obvious that no explanation is needed, or if there are no differences in the code (e.g., documentation additions), you can skip this section.  
> Delete this sentence and describe this pull request.  
> For example, it is desirable to describe the specifications of added functions, and detailed explanations of bugs that have been fixed.  

> [!IMPORTANT]
> The vehicle models in `simulation/simple_sensor_simulator/include/simple_sensor_simulator/vehicle_simulation/vehicle_model` and `simulation/simple_sensor_simulator/src/vehicle_simulation/vehicle_model` are copied from [simple_planning_simulator](https://github.com/autowarefoundation/autoware_simple_planning_simulator). If this pull request changes them, consider making the changes to the original code first.

## References

> [Optional] If the referenced material does not exist, you can skip this section.  
> Describe any standards, algorithms, books, articles, etc. that you referenced when creating the pull request. If there is any novelty, mention it.  

# Destructive Changes

> [Optional] If no destructive change exists, you can skip this section.  
> Include a description of the changes and a migration guide and send the pull request with a bump major label. (Example : https://github.com/tier4/scenario_simulator_v2/pull/1139)  
> Otherwise, skip the "Destructive Changes" section and make sure this pull request is labeled `bump minor` or `bump patch`.  

> [!CAUTION]
> Changes to the message definitions in `openscenario/openscenario_interpreter_msgs/msg` and `simulation/traffic_simulator_msgs/msg` may seriously break the integration with external services such as Web.Auto. If this pull request contains such changes, describe them in this section.

# Known Limitations

> [Optional] If there are no known limitations that you can think of, you can skip this section. If there are any limitations on the features or fixes added by this pull request, delete this sentence and clearly describe them.
> For example, the lane matching algorithm currently (1/25/2024) employed is unable to match Entity that is heavily tilted with respect to the lane, and it is difficult to throw an exception.  
> If the developer is aware of the existence of such problems or limitations, describe them in detail. The problems or limitation should be listed as an Issue on GitHub and a link to it should be provided in this section.  

# Checklist for reviewers

All references to "You" in the following text refer to the code reviewer.

- [ ] Is this pull request written in a way that is easy to read from a third-party perspective?
- [ ] Is there sufficient information (background, purpose, specification, algorithm description, list of disruptive changes, and migration guide) in the description of this pull request?
- [ ] If this pull request contains a destructive change, does this pull request contain the migration guide?
- [ ] All unit tests/integration tests are included in this pull request? If you think adding test cases is unnecessary, please describe why and cross out this line.
- [ ] The documentation for this pull request is enough? If you think adding documents for this pull request is unnecessary, please describe why and cross out this line.

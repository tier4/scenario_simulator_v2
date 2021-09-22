# Architecture design

![Scenario Testing Framework](../image/what_is_scenario_testing_framework.png "what is scenario testing framework")

This documentation describes the architecture design of this scenario testing framework.

This framework is designed for executing scenario tests with Autoware.

<font color="#065479E">**This framework is designed to easily accommodate multiple simulators and scenario description formats.**</font>

## Simple sensor simulator

If you want to know about our very very simple lidar simulator, please read [this documentation](SimpleSensorSimulator.md).

## Traffic simulator

If you want to know how to control traffics in simulation, and how to develop your original scenario format, please read [this documentation](TrafficSimulator.md).

## Communication with simulator and interpreter

If you want to know how to connect your simulator with this framework, please read [this documentation](ZeroMQ.md).

## Autoware API

If you want to know how we can send commands to the Autoware via simulators, please read [this documentation](AutowareAPI.md).

## System architecture

If you want to know whole system architecture of this framework, please read [this documentation](SystemArchitecture.md).

## TierIV Scenario Format Version 2.0

If you want to know TierIV Scenario format Version 2.0 , which is a default scenario format of this framework, please read [this documentation](TierIVScenarioFormatVersion2.md).

## How to implement ORCA Plugin
### Choose Plugin type
There are 2 types of plugin as below:
- Obstacle ORCA Plugin
- Agent ORCA Plugin

Usually use the Obstacle plugin.

### Implement Plugin Class
- Make plugin class
  - Inherit base class in `include/Plugins/OrcaPlugin.h`
    - `OrcaAgentPluginBase` class
    - `OrcaObstaclePluginBase` class
  - Implement Constructor method
    - Call the constructor method of the base class with plugin name.
  - Implement `calcOrcaLines` method (override)
    - Note that the arguments differ depending on the type of plugin
    - Add the activation implementation at the beggining of the method as follows:
```c++
std::vector<RVO::Line> HogePlugin::calcOrcaLines(...)
{
  std::vector<RVO::Line> orca_lines;
  // skip if inactive
  if(!is_active_){
    return orca_lines;
  }
  // plugin implementation
}
```

### Use custom plugins
- Make instance of the custom plugin as a shared pointer
- Add plugins to agent using methods below:
  - `Agent::addAgentORCAPlugin`
  - `Agent::addObstacleORCAPlugin`
- You can switch activation with `activate` or `deactivate` methods

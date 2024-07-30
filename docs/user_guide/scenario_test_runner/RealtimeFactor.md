# How to use realtime factor

It is possible to modify the speed of simulation (the speed of time published on the /clock topic is not affected):

- from the start of the simulation (using parameter),
- during the simulation (using the GUI slider).


!!! Warning
    Parameter `use_sim_time` of `openscenario_interpreter` is false by default and should not be modified.

    | use_sim_time launch parameter | /clock time published by SS2 | AWF Autoware Time  |
    |-------------------------------|------------------------------|---------------------|
    | false (default)               | walltime                     | walltime            |
    | true (not supported)          | -                            | -                   |


## Use parameter

 - When you run simulations on the command line, add an `global_real_time_factor`  parameter with a custom value (the default is 1.0). 

   ```bash
   ros2 launch scenario_test_runner scenario_test_runner.launch.py \
   architecture_type:=awf/universe \
   record:=false \
   scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample.yaml' \
   sensor_model:=sample_sensor_kit \
   vehicle_model:=sample_vehicle \
   global_real_time_factor:="0.5"
   ``` 

 - The smaller the value you specify, the slower the simulation will progress.

## Use slider on run time 

- When the simulation is started you can add the `RViz` panel by clicking `Panels -> Add new panel` in the top left corner of RViz.

- Then in the pop up window please select `RealTimeFactorSliderPanel` and double click on it.

![Panel](../../image/realtime_factor/panel.png)

- Slider controlling the speed of simulation time should be visible on the left side of the screen.

![Slider](../../image/realtime_factor/slider.png)

- The process of adding the panel is also visible in the video:

<video width="1080" controls muted>
    <source src="/image/realtime_factor/video.mp4" type="video/mp4">
</video>
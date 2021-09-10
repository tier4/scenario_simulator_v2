# Simulation Result Format
## File format
Result file should be in junit format.  
[This package](https://github.com/tier4/scenario_simulator_v2/tree/master/common/simple_junit) helps to output junit.
Example of result file is here.  

```xml
<?xml version="1.0"?>
<testsuites name="/tmp/scenario_test_runner">
  <testsuite name="TESTSUITE_NAME">
    <testcase name="TESTCASE_NAME"/>
  </testsuite>
</testsuites>
```

If you want to know about junit format, please refer [this](https://help.catchsoftware.com/display/ET/JUnit+Format).

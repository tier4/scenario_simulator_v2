# Development Rules

If you have some questions and opinion, please feel free to contact maintainers in [github issue.](https://github.com/tier4/scenario_simulator_v2/issues)

## Branch Naming
### Feature development

Please use "feature" prefix.  
Branch name should be like below.

```
feature/(name of the feature you are developing.)
```

feature branch will be merged into master branch and deleted.

### Bugfix

Please use "fix" prefix.
For example,

```
fix/(name of bug what you are fixing.)
```

fix branch will be merged into master branch and deleted.

### Release
Only maintainers create this branch.  
Nameing rules are below.  

```
release/prepare_(version_tag)
```

This branch only update release notes.  
For example, [link](https://github.com/tier4/scenario_simulator_v2/pull/477)


## Merge rule
### Continuous Integration

If you modify source code, you should be pass all checks below.

|                                                                                                 Budge                                                                                                  |                          Description                           |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------- |
| [![ScenarioTest](https://github.com/tier4/scenario_simulator_v2/actions/workflows/ScenarioTest.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/ScenarioTest.yaml)    | Build all packages and runnnig integration test.               |
| [![BuildTest](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Build.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Build.yaml)                     | Build each package independentry and run linter and unit test. |
| [![Docker](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Docker.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Docker.yaml)                      | Build docker image.                                            |
| [![Documentation](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml) | Build documentation site.                                      |
| [![SpellCheck](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml)          | Run spell checker and leave comment.                           |




Currently, spellcheck is not required, but please check and fix typo if possible.

If you modify only documentation, you should pass checks below.

|                                                                                                 Budge                                                                                                  |                          Description                           |
| ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------- |
| [![Documentation](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml) | Build documentation site.                                      |
| [![SpellCheck](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml)          | Run spell checker and leave comment.                           |

### Code review
More than one [maintiner](/etc/ContactUs) approve your code and check all test cases are pased, your code will be merged into master branch.  
Your contribution will be recorded in [release note.](/ReleaseNotes)

### How to merge your contribution
Each branches create merge commit and merge into master branch.

## Push rule
If you push your source code to scenario_simulator_v2, please follow these guidelines.

1. If you are try to solving urgent problem which only can be solve with force push, please contact to the maintainer and aftter the maintainer allowed it, then you can execute force push.
1. Anyone can execute force push to the master branch.
1. Pushing your source code to the master branch is principle prohibited.
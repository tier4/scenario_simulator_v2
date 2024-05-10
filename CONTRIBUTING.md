# How to contribute

Thank you for your interest in improving scenario_simulator_v2.
This contributor guidelines will help you get started.

If you have any questions, please feel free to make GitHub issues for asking your question.

## Before you get started

To avoid duplicate work, please review current [issues](https://github.com/tier4/scenario_simulator_v2/issues) and [pull requests](https://github.com/tier4/scenario_simulator_v2/pulls).

## Contribution workflow

Please take the following steps:

1. Fork this repository.
2. Clone down the repository to your local machine.
3. Create a new branch from the `master`. See [Branch naming rules](#branch-naming-rules) below.
4. Commit your changes to the branch and push the commit to your GitHub repository that was forked in Step 1.
5. Open a Pull Request (PR). Our CI workflows will automatically test your changes. See [Continuous integration](#continuous-integration) below.
6. The maintainers will review your PR. Once the PR is approved, the code will be merged into the `master` branch. See [Code review](#code-review) below.

## Branch naming rules

### Feature development

Each new feature development should be worked on in its own branch.

Please add the `feature/` prefix to your branch name. For example:

```
feature/(name of the feature you are developing)
```

### Bugfix

Please add the `fix/` prefix to your branch name. For example:

```
fix/(name of bug what you are fixing)
```

### Release

Only maintainers create the `release/` branch.

```
release/prepare_(version_tag)
```

The release branches are used only to update the release notes. An example is [here](https://github.com/tier4/scenario_simulator_v2/pull/477).

## Continuous integration

Your changes proposed in your pull request will be tested automatically by the following checks:

| Checks                                                                                                                                                                                                                            | Description                                                                          |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| [![BuildAndRun](https://github.com/tier4/scenario_simulator_v2/actions/workflows/BuildAndRun.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/BuildAndRun.yaml)                                  | Build each package independently, run linters, unit tests and scenario tests.        |
| [![CheckBranchUpToDate](https://github.com/tier4/scenario_simulator_v2/actions/workflows/CheckBranchUpToDate.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/CheckBranchUpToDate.yaml)          | Checking the branch is up to date. This workflow works on merge queue.               |
| [![CheckLabel](https://github.com/tier4/scenario_simulator_v2/actions/workflows/CheckLabel.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/CheckLabel.yaml)                                     | Checking the label for version control is labeled.                                   |
| [![Docker](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Docker.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Docker.yaml)                                                 | Build a docker image.                                                                |
| [![Documentation](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml)                            | Build the documentation sites.                                                       |
| [![DocumentationLinkCheck](https://github.com/tier4/scenario_simulator_v2/actions/workflows/DocumentationLinkCheck.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/DocumentationLinkCheck.yaml) | Checking the URLs in documentation are valid.                                        |
| [![LineLint](https://github.com/tier4/scenario_simulator_v2/actions/workflows/LineLint.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/LineLint.yaml)                                           | Checking text files contain the blank line at the end of the files.                  |
| [![Release](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Release.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Release.yaml)                                              | Bump new version and create a release. This workflow works on merge queue.           |
| [![SpellCheck](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml)                                     | Run a spell checker and add warnings to the PR.                                      |

If you contribute to the documentation, your changes should pass the checks below:

| Checks                                                                                                                                                                                                                            | Description                                                              |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| [![CheckBranchUpToDate](https://github.com/tier4/scenario_simulator_v2/actions/workflows/CheckBranchUpToDate.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/CheckBranchUpToDate.yaml)          | Checking the branch is up to date. This workflow works on merge queue.   |
| [![CheckLabel](https://github.com/tier4/scenario_simulator_v2/actions/workflows/CheckLabel.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/CheckLabel.yaml)                                     | Checking the label for version control is labeled.                       |
| [![Documentation](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Documentation.yaml)                            | Build the documentation sites.                                           |
| [![DocumentationLinkCheck](https://github.com/tier4/scenario_simulator_v2/actions/workflows/DocumentationLinkCheck.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/DocumentationLinkCheck.yaml) | Checking the URLs in documentation are valid.                            |
| [![Release](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Release.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/Release.yaml)                                              | Bump new version and create a release. This workflow works on merge queue. |
| [![SpellCheck](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml/badge.svg)](https://github.com/tier4/scenario_simulator_v2/actions/workflows/SpellCheck.yaml)                                     | Run a spell checker and add warnings to the PR.                          |

## Code review

Any changes to the code or documentation are subject to code review. Maintainers will review your pull request.

As a good practice, reply to the reviewer's comment with a link to your changes (e.g., `Fixed in a0b1c2d`).
To keep the commit hashes consistent, **please DO NOT force-push the commit to your pull request during the code review.**
If you want to force-push the commit during the review, please contact the maintainers for approval in advance.

If at least one maintainer approves your pull request and all checks are passed, your pull request will be merged into the `master` branch.
Your contribution will be recorded in the [release note](https://tier4.github.io/scenario_simulator_v2-docs/release/ReleaseNotes/).

Pull requests must be labeled `bump major`, `bump minor` or `bump patch`.

Please follow the criteria below to determine which label to apply.

* If there is a destructive change to the scenario or traffic_simulator API, label it label it with `bump major`.
* If the scenario or traffic_simulator API has some additional functionality but no destructive changes, label it with `bump minor`.
* If there is no additional functionality and full backward compatibility is ensured, label it with `bump patch`.

If you are in any doubt, please consult the maintainers [@hakuturu583](https://github.com/hakuturu583),[@yamacir-kit](https://github.com/yamacir-kit),[@HansRobo](https://github.com/HansRobo).

## Update Documentation

If your pull request changes the behavior of the software, please update the documentation in the `docs/` directory.
The documentation is built by [mkdocs](https://www.mkdocs.org/).

You may rename existing files or add new files to the `docs/` directory.
* If you add new files, you should add the file name to the `nav` section in the `mkdocs.yml` file.

* If you rename existing files, you should set up a redirect from the old file to the new file.
Redirects are defined in the `mkdocs.yml` file using `mkdocs-redirects` plugin, so you will edit `plugins.redirects.redirect_maps` in the `mkdocs.yml` file.

## License

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

> 5. Submission of Contributions. Unless You explicitly state otherwise,
     >    any Contribution intentionally submitted for inclusion in the Work
     >    by You to the Licensor shall be under the terms and conditions of
     >    this License, without any additional terms or conditions.
     >    Notwithstanding the above, nothing herein shall supersede or modify
     >    the terms of any separate license agreement you may have executed
     >    with Licensor regarding such Contributions.

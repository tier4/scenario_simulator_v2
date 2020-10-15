#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 Autoware Foundation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import argparse
from ament_index_python.packages import get_package_share_directory
import yamale

class WorkflowValidator():
    def __init__(self):
        share_directory_path = os.path.join(get_package_share_directory('scenario_test_utility'))
        schema_path = share_directory_path + 
                      '/../ament_index/resource_index/packages/workflow_schema.yaml'
        print(schema_path)
        self.workflow_schema = yamale.make_schema(schema_path)
    def validate_workflow_file(self):
        pass

def main():
    parser = argparse.ArgumentParser(description='Validator for workflow .yaml file')
    parser.add_argument('workflow', help='path to workflow .yaml file')
    args = parser.parse_args()
    validator = WorkflowValidator()
    validator.validate_workflow_file(args.workflow)


if __name__ == '__main__':
    """Entrypoint."""
    main()
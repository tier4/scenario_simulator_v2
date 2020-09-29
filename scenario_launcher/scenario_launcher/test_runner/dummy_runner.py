#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 Tier IV, Inc. All rights reserved.
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

import random
import time
import xmlrpc.client


class DummyRunner:

    SERVER_URI = 'http://0.0.0.0:10000'
    SLEEP_RATE = 1

    def __init__(self):
        self.client = None

    def test_run(self, timeout, exitcode):
        start = time.time()
        traveled_distance = 0
        self.client.set_simulation_running(True)
        while (time.time() - start) < timeout:
            current_time = time.time() - start
            traveled_distance = traveled_distance + 10
            self.client.update_simulation_time(current_time)
            self.client.update_traveled_distance(traveled_distance)
            time.sleep(self.SLEEP_RATE)
        self.client.set_simulation_running(False)
        self.client.set_exit_code(exitcode)

    def main(self):
        self.client = xmlrpc.client.Server(self.SERVER_URI)
        i = random.randint(0, 3)
        timeouts = [5, 6, 11, 12]
        exitcodes = [1, 0, 200, 201]
        self.test_run(timeouts[i], exitcodes[i])
        time.sleep(1)


def main():
    drunner = DummyRunner()
    drunner.main()


if __name__ == "__main__":
    main()

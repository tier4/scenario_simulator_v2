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

import time
import xmlrpc.client


class TestClient:

    SERVER_URI = 'http://0.0.0.0:9999'
    SLEEP_RATE = 1

    def __init__(self):
        print("initialize test runner")
        self.init_client()

    def init_client(self):
        self.client = xmlrpc.client.Server(self.SERVER_URI)

    def test_run(self, timeout, exitcode, traveled_distance):
        start = time.time()
        while (time.time() - start) < timeout:
            current_time = time.time() - start
            self.client.update_simulation_time(current_time)
            self.client.update_traveled_distance(traveled_distance)
            time.sleep(self.SLEEP_RATE)
        print("")
        self.client.set_exit_code(exitcode)

    def main(self):
        timeouts = [1, 2, 3, 4]
        exitcodes = [1, 0, 200, 201]
        traveled_distances = [0, 10, 20, 40]
        for i in range(4):
            self.test_run(timeouts[i], exitcodes[i], traveled_distances[i])
        time.sleep(1)


def main():
    drunner = TestClient()
    drunner.main()


if __name__ == "__main__":
    main()

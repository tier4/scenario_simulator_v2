#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
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

# Modifications Copyright 2022 TIER IV, Inc.

"""Module for the Shutdown action."""

import logging
from typing import Text

from launch.frontend import Entity, expose_action, Parser

from launch.actions import EmitEvent
from launch.events import Shutdown as ShutdownEvent
from launch.events.process import ProcessExited
from launch.launch_context import LaunchContext

_logger = logging.getLogger(name="launch")


class ShutdownOnce(EmitEvent):
    shutdown_called = False
    """Action that shuts down a launched system by emitting Shutdown when executed."""

    def __init__(self, *, reason: Text = "reason not given", **kwargs):
        super().__init__(event=ShutdownEvent(reason=reason), **kwargs)

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Return `Shutdown` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        reason = entity.get_attr("reason", optional=True)
        if reason:
            kwargs["reason"] = parser.parse_substitution(reason)
        return cls, kwargs

    def execute(self, context: LaunchContext):
        """Execute the action."""
        if ShutdownOnce.shutdown_called:
            return
        else:
            ShutdownOnce.shutdown_called = True
            try:
                event = context.locals.event
            except AttributeError:
                event = None

            if isinstance(event, ProcessExited):
                _logger.info(
                    "process[{}] was required: shutting down launched system".format(
                        event.process_name
                    )
                )

            super().execute(context)

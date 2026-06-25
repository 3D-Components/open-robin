"""Shared helpers for lifecycle node launch files."""

from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def lifecycle_autostart(node):
    """Return launch entities that auto-configure then activate *node*.

    Usage::

        return LaunchDescription(lifecycle_autostart(my_node) + [other_actions])
    """
    activate_on_configure = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
            handle_once=True,
        )
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    return [activate_on_configure, node, configure]

"""Target plugin interfaces for RoboFuzz."""

from __future__ import annotations


class BaseTargetPlugin:
    """Base hook interface for target-specific behavior.

    Subclasses can override any hook as needed.
    """

    def __init__(self, target_config: dict, runtime_config) -> None:
        self.target_config = target_config
        self.runtime_config = runtime_config

    def pre_exec_hook(self, msg):
        """Mutate/adjust the outgoing message before publish."""
        return msg

    def post_exec_hook(self) -> None:
        """Called after one execution finishes."""
        return None

    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        """Run target-specific oracle checks and return error strings."""
        return []

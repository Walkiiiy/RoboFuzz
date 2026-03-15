from target_plugins import BaseTargetPlugin
import oracles.rosidl


class TargetPlugin(BaseTargetPlugin):
    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        return oracles.rosidl.check(config, msg_list, state_dict, feedback_list)

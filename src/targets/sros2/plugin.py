from target_plugins import BaseTargetPlugin


class TargetPlugin(BaseTargetPlugin):
    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        return []

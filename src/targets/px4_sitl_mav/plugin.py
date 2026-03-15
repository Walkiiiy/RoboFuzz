from target_plugins import BaseTargetPlugin
import oracles.px4


class TargetPlugin(BaseTargetPlugin):
    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        return oracles.px4.check(config, msg_list, state_dict, feedback_list)

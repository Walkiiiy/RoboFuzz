from target_plugins import BaseTargetPlugin
import oracles.turtlesim


class TargetPlugin(BaseTargetPlugin):
    def check_oracle(self, config, msg_list, state_dict, feedback_list):
        pose_list = state_dict.get("/turtle1/pose", [])
        return oracles.turtlesim.check(config, msg_list, pose_list, feedback_list)

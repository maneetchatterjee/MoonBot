from moonbot.environments.base import MoonBotEnv


class ArmReachingEnv(MoonBotEnv):
    def __init__(self, **kwargs: object) -> None:
        super().__init__(task="arm_reaching", **kwargs)

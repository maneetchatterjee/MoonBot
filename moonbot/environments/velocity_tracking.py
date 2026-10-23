from moonbot.environments.base import MoonBotEnv


class VelocityTrackingEnv(MoonBotEnv):
    def __init__(self, **kwargs: object) -> None:
        super().__init__(task="velocity_tracking", **kwargs)

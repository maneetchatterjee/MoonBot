from moonbot.environments.base import MoonBotEnv


class MobileManipulationEnv(MoonBotEnv):
    def __init__(self, **kwargs: object) -> None:
        super().__init__(task="mobile_manipulation", **kwargs)

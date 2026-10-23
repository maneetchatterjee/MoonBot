from moonbot.environments.base import MoonBotEnv


class WaypointNavigationEnv(MoonBotEnv):
    def __init__(self, **kwargs: object) -> None:
        super().__init__(task="waypoint_navigation", **kwargs)

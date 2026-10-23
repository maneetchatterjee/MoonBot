from pathlib import Path

import pytest


@pytest.fixture(scope="session")
def project_root() -> Path:
    return Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def model_path(project_root: Path) -> Path:
    return project_root / "assets" / "mjcf" / "moonbot_x.xml"

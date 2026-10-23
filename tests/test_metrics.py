import numpy as np
import pytest

from moonbot.evaluation.metrics import bootstrap_mean_interval, root_mean_square


def test_rmse():
    assert root_mean_square(np.array([3.0, 4.0])) == pytest.approx(np.sqrt(12.5))


def test_rmse_rejects_empty_input():
    with pytest.raises(ValueError):
        root_mean_square(np.array([]))


def test_bootstrap_is_reproducible_and_ordered():
    values = np.array([0.0, 1.0, 2.0, 3.0])
    first = bootstrap_mean_interval(values, samples=500, seed=7)
    second = bootstrap_mean_interval(values, samples=500, seed=7)
    assert first == second
    assert first[0] <= np.mean(values) <= first[1]

"""Trivial example test to verify the test suite is wired up correctly."""

from dingo_project.example import add


def test_add() -> None:
    """add() should return the sum of its two arguments."""
    assert add(2, 3) == 5

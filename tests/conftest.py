import itertools

import pytest

_can_ids = itertools.count(40)


@pytest.fixture
def can_id():
    """Fresh CAN ID per call, so sim devices from different tests never collide."""
    return lambda: next(_can_ids)

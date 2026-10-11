import pytest


def pytest_collection_modifyitems(config, items):
    skip = pytest.mark.skip(reason="checks the finished exercises; the public version ships TODO stubs")
    for item in items:
        if "solution" in item.keywords:
            item.add_marker(skip)

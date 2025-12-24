"""
Utility subpackage for synria_common_sdk.

Contains:
- singleton: simple Singleton base class
- bqueue: bounded thread-safe queue
"""

from .singleton import Singleton  # noqa: F401
from .bqueue import BQueue        # noqa: F401



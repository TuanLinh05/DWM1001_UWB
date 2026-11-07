"""Small, dependency-free helpers shared by the offline G0 reports."""

from dataclasses import asdict, is_dataclass
import math
from collections.abc import Mapping


def number(value, default=None):
    try:
        out = float(value)
        return out if math.isfinite(out) else default
    except (ValueError, TypeError, OverflowError):
        return default


def strict_value(value):
    if is_dataclass(value):
        value = asdict(value)
    if isinstance(value, Mapping):
        return {str(k): strict_value(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [strict_value(v) for v in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    return value

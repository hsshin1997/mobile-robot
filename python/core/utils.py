"""Small math helpers shared across layers."""
from __future__ import annotations
import numpy as np
from matplotlib.patches import Ellipse

__all__ = ["wrap_to_pi", "cov_to_ellipse"]

def wrap_to_pi(angle: float | np.ndarray) -> float | np.ndarray:
    """Map radians to (−π, π]. Works with scalars or NumPy arrays."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def cov_to_ellipse(P: np.ndarray, n_sigma: float = 2.0):
    """Return width, height, angle° of the n‑σ covariance ellipse (2×2 slice)."""
    vals, vecs = np.linalg.eigh(P)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    width, height = 2 * n_sigma * np.sqrt(vals)
    angle = np.degrees(np.arctan2(vecs[1, 0], vecs[0, 0]))
    return width, height, angle
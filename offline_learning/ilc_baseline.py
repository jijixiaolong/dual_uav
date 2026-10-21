"""Conventional event-aligned Iterative Learning Control (ILC) baseline."""
import numpy as np


class EventAlignedILC:
    """Iterative Learning Control baseline updated across repeated trials."""

    def __init__(self, n_basis: int = 4, learning_gain: float = 0.05, max_limit: float = 1.0):
        self.n_basis = n_basis
        self.gamma = learning_gain
        self.max_limit = max_limit
        self.c = np.zeros(n_basis, dtype=np.float64)
        self.history = [self.c.copy()]

    def reset(self):
        self.c = np.zeros(self.n_basis, dtype=np.float64)
        self.history = [self.c.copy()]

    def update(self, G: np.ndarray, y_error: np.ndarray):
        """Update feedforward waveform using gradient of lifted tracking error."""
        # Gradient step: delta_c = -gamma * G^T * y
        grad = G.T @ y_error
        self.c -= self.gamma * grad
        self.c = np.clip(self.c, -self.max_limit, self.max_limit)
        self.history.append(self.c.copy())
        return self.c.copy()

    def get_coefficients(self) -> np.ndarray:
        return self.c.copy()


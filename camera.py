"""
File to contain the Camera class
"""

import numpy as np

class Camera:
    def __init__(self) -> None:
        self._view = np.zeros((100,100))

    @property
    def view(self):
        return self._view

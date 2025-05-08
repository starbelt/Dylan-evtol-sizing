import json
import abc  # For abstract base classes
from typing import Dict, Any

# Abstract base class for all aircraft components.
class AircraftComponent(abc.ABC): # Inherit from abc.ABC to allow abstract methods
    def __init__(self, path_to_json: str):
        self.path_to_json = path_to_json
        self._varlist: Dict[str, Any] = {}
        self.weight: float = 0.0

    def load_variables_from_json(self):
        with open(self.path_to_json, "r") as f:
            self._varlist = json.load(f)
            
    @abc.abstractmethod
    def UpdateComponent(self, aircraft_state: dict):
        self.load_variables_from_json()

    def get_weight_kg(self) -> float:
        return self.weight
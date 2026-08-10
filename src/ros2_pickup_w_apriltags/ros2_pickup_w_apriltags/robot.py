from .arm import ArmController
from .base import BaseController

class Robot:

    def __init__(self, node):
        self.arm = ArmController(node)
        self.base = BaseController(node)
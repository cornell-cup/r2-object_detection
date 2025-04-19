import sys, os, logging # Default Python Libraries

from typing import List, Tuple # Type Hinting

# Computer Architecture Config.
DEFAULT_OPEN: bool      = os.getenv('DEFAULT_OPEN', True) # Decides Whether To Open Camera
DEFAULT_DISP: bool      = os.getenv('DEFAULT_DISP', True) # Decides Whether To Display Images
DEFAULT_PRINT: bool     = os.getenv('DEFAULT_PRINT', True) # Decides Whether To Print Intermediates
DEFAULT_CAMERA: str     = os.getenv('DEFAULT_CAMERA', None) # Camera Index / Identifier

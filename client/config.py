import sys, os, logging # Default Python Libraries

from typing import List, Tuple # Type Hinting

# Computer Architecture Config.
DEFAULT_PATH: str       = os.getenv('DEFAULT_PATH', 'resources/people') # Path To Image Directory.
DEFAULT_OPEN: bool      = os.getenv('DEFAULT_OPEN', True) # Decides Whether To Open Camera
DEFAULT_LOAD: bool      = os.getenv('DEFAULT_LOAD', True) # Decides Whether To Load Cache
DEFAULT_DISP: bool      = os.getenv('DEFAULT_DISP', True) # Decides Whether To Display Images
DEFAULT_PRINT: bool     = os.getenv('DEFAULT_PRINT', True) # Decides Whether To Print Intermediates
DEFAULT_CACHE: bool     = os.getenv('DEFAULT_CACHE', True) # Decides Whether To Cache Images
DEFAULT_CACHE_DIR: str  = os.getenv('DEFAULT_CACHE_DIR', '.cache') # Path To Cache Directory
DEFAULT_CAMERA: str     = os.getenv('DEFAULT_CAMERA', None) # Camera Index / Identifier

# Facial Recognition Config.
COLORS: List[Tuple[int, int, int]] = [(0, 0, 255), (0, 255, 0), (255, 0, 0)] # Bounding Box Colors
DEFAULT_SCALE_FACTOR: float        = 0.5 # Scale Factor For Image Resizing
DEFAULT_NUM_JITTERS: int           = 2 # Number Of Noisy Images To Generate
DEFAULT_NUM_UPSAMPLE: int          = 1 # Number Of Upsamples To Perform
DEFAULT_ENCODING_MODEL: str        = 'large' # Either 'large' or 'small'
DEFAULT_NN_MODEL: str              = 'cnn' # Either 'cnn' or 'hog'
DEFAULT_UNKNOWN_FACE_ID: str       = 'Unknown' # Unknown Face Identifier
TOLERANCE: float                   = 0.5 # Tolerance For Matching Faces
IMG_EXTs: List[str]                = ['jpg', 'jpeg', 'png'] # Allowed Image Extensions
ENCODING_EXT: str                  = 'enc' # Encoding File Extension
TEXT_ENCODING: str                 = 'utf-8' # Text Encoding Format

#from importlib.metadata import version
#__version__ = version(__name__)

from importlib.metadata import version, PackageNotFoundError

try:
    __version__ = version(__name__)
except PackageNotFoundError: 
    # Using a placeholder version. 
    __version__ = "0.0.0+local"

import sys
from pathlib import Path

# serial_log.py sits in cart/, not in a package
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

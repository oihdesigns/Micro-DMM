"""Use the app-local runtime without changing the user's Python installation."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent
RUNTIME = ROOT / '.runtime'
if RUNTIME.exists():
    sys.path.insert(0, str(RUNTIME))

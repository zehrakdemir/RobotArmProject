"""Testler paket KURULMADAN da kosabilsin diye src/ sys.path'e eklenir.

`pip install -e .` yapilmis bir ortamda bu zararsiz (zaten bulunur); ama
CI'da ve hizli bir yerel kontrolde kurulum adimini beklemeden test
kosturabilmek isteniyor. pytest'in `pythonpath` ini secenegi pytest 7+
gerektiriyor, bu makinedeki sistem pytest'i daha eski - bu yuzden burada.
"""

import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

"""Shared matplotlib setup. Uses alex_style if ALEX_STYLE_PATH points at its folder."""

import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

_style = None
if os.environ.get("ALEX_STYLE_PATH"):
    sys.path.insert(0, os.environ["ALEX_STYLE_PATH"])
    import alex_style as _style  # noqa: E402

    _style.use("dark")


def title(axes, text, caption):
    if _style:
        _style.title(axes, text, caption)
    else:
        axes.set_title(f"{text}\n{caption}", loc="left")


def save(fig, path):
    if _style:
        _style.save(fig, path)
    else:
        fig.tight_layout()
        fig.savefig(path, dpi=150)
    plt.close(fig)

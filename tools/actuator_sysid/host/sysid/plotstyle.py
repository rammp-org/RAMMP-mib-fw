"""The one plot style every analysis uses: alex-style, dark (these plots are read on screen).

alex_style.py is loaded from $ALEX_STYLE_DIR or ~/code-stuff/alex-style-guide/matplotlib.
On a machine without the guide the same palette and rules are applied from the
copy below, so the tools still run (fonts fall back to Helvetica/DejaVu).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

_DIRS = [os.environ.get("ALEX_STYLE_DIR", ""), str(Path.home() / "code-stuff/alex-style-guide/matplotlib")]
ax_style = None
for _d in _DIRS:
    if _d and (Path(_d) / "alex_style.py").exists():
        sys.path.insert(0, _d)
        try:
            import alex_style as ax_style  # type: ignore
        except Exception:  # pragma: no cover
            ax_style = None
        break

_FALLBACK = {
    "bg": "#0e0f10", "panel": "#141516", "ink": "#e4e1d8", "muted": "#8a877e", "grid": "#26282b",
    "accent": "#e2683c", "series": ["#2f62e8", "#e03a2f", "#0f9a66", "#a45ee0", "#1a9cc0", "#d9539a"],
    "good": "#4fbf5a", "warning": "#e6c229", "critical": "#ff5a4a", "off": "#5a5c5e",
}

DPI = 140
_ready = False


def setup(mode: str = "dark"):
    global _ready
    if _ready:
        return
    if ax_style is not None:
        ax_style.use(mode)
    else:
        p = _FALLBACK
        plt.rcParams.update({
            "font.family": "sans-serif", "font.sans-serif": ["Roboto", "Helvetica Neue", "Arial", "DejaVu Sans"],
            "font.size": 10, "text.color": p["ink"], "figure.facecolor": p["bg"], "savefig.facecolor": p["bg"],
            "axes.facecolor": p["panel"], "axes.edgecolor": "#4a4c4e", "axes.labelcolor": p["ink"],
            "axes.labelsize": 9, "axes.spines.top": False, "axes.spines.right": False, "axes.spines.left": False,
            "axes.grid": True, "axes.grid.axis": "y", "axes.axisbelow": True, "grid.color": p["grid"],
            "grid.linestyle": (0, (2, 4)), "grid.linewidth": 0.8, "xtick.color": p["muted"],
            "ytick.color": p["muted"], "xtick.labelsize": 8.5, "ytick.labelsize": 8.5, "ytick.left": False,
            "lines.linewidth": 2.0, "lines.solid_joinstyle": "round", "legend.frameon": False,
            "legend.fontsize": 8.5, "legend.labelcolor": p["muted"], "savefig.bbox": "tight",
            "axes.prop_cycle": plt.cycler(color=p["series"]),
        })
    _ready = True


def colors() -> dict:
    return ax_style.colors() if ax_style is not None else _FALLBACK


def series(i: int) -> str:
    s = colors()["series"]
    return s[i] if i < len(s) else colors()["muted"]


def title(a, text, sub=None):
    if ax_style is not None:
        ax_style.title(a, text, sub)
        return
    p = colors()
    a.set_title(text.upper(), loc="left", color=p["ink"], fontsize=11, fontweight="bold", pad=22 if sub else 10)
    if sub:
        a.text(0, 1.02, sub, transform=a.transAxes, color=p["muted"], fontsize=8.5, va="bottom")


def label_lines(a, min_gap=None):
    if ax_style is not None:
        ax_style.label_lines(a, min_gap)


def bars(a, labels, values, highlight=None, unit="", xlabel=None, ylabel=None):
    if ax_style is not None:
        ax_style.bars(a, labels, values, highlight=highlight, unit=unit, xlabel=xlabel, ylabel=ylabel)
        return
    p = colors()
    cols = [p["series"][1] if lab == highlight else p["series"][0] for lab in labels]
    y = range(len(labels))
    a.barh(y, values, color=cols, height=0.55)
    a.set_yticks(list(y), labels)
    a.invert_yaxis()
    a.grid(axis="y", visible=False)
    a.grid(axis="x", visible=True)
    for i, v in enumerate(values):
        a.annotate(f"{v}{unit}", xy=(v, i), xytext=(5, 0), textcoords="offset points", va="center",
                   color=p["muted"], fontsize=8.5)
    if xlabel:
        a.set_xlabel(xlabel)
    if ylabel:
        a.set_ylabel(ylabel)


def ref_line(a, value, label, status="warning", vertical=False, bottom=False):
    p = colors()
    if vertical:
        a.axvline(value, color=p[status], linestyle=(0, (4, 3)), linewidth=1)
        a.annotate(label, xy=(value, 0 if bottom else 1), xycoords=("data", "axes fraction"),
                   xytext=(3, 4 if bottom else -10), textcoords="offset points", color=p["muted"], fontsize=7.5)
    elif ax_style is not None:
        ax_style.ref_line(a, value, label, status)
    else:
        a.axhline(value, color=p[status], linestyle=(0, (4, 3)), linewidth=1)
        a.annotate(label, xy=(0, value), xycoords=("axes fraction", "data"), xytext=(4, 4),
                   textcoords="offset points", color=p["muted"], fontsize=8)


def legend_above(a, items, inside=False):
    """Square-swatch legend above the plot, right aligned. items: [(label, color)].
    inside=True puts it in the plot's upper right when the subtitle is long."""
    handles = [Patch(facecolor=c, edgecolor="none", label=l) for l, c in items]
    if inside:
        a.legend(handles=handles, loc=inside if isinstance(inside, str) else "upper right", ncol=min(len(items), 3), frameon=False, handlelength=0.8,
                 handleheight=0.8, fontsize=8)
        return
    a.legend(handles=handles, loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=len(items), frameon=False,
             handlelength=0.8, handleheight=0.8, borderaxespad=0.2, columnspacing=1.0, fontsize=8)


def mark_events(axes, events, label_axis=None, max_labels=30):
    """Thin dashed muted verticals at each event; short labels on the top axis."""
    p = colors()
    for a in axes:
        for e in events:
            a.axvline(e.t, color=p["off"], linewidth=0.7, linestyle=(0, (2, 3)), zorder=0)
    if label_axis is not None and events:
        step = max(1, len(events) // max_labels)
        for e in events[::step]:
            label_axis.annotate(e.text[:22], xy=(e.t, 1.0), xycoords=("data", "axes fraction"), xytext=(2, -2),
                                textcoords="offset points", rotation=90, va="top", ha="left", fontsize=6.5,
                                color=p["muted"], annotation_clip=True)


def figure(nrows=1, ncols=1, w=11.0, h_per=2.4, sharex=False, **kw):
    setup()
    fig, axs = plt.subplots(nrows, ncols, figsize=(w, max(2.6, h_per * nrows)), sharex=sharex, squeeze=False,
                            constrained_layout=True, **kw)
    return fig, axs


def page_title(fig, text, meta=""):
    """Terracotta square, then the uppercase title; run metadata on the right, muted mono."""
    from matplotlib.transforms import offset_copy

    p = colors()
    have = ax_style is not None
    fig.text(0.0, 1.01, "\u25a0", color=p["accent"], fontsize=12, va="bottom", ha="left", fontfamily="DejaVu Sans")
    fig.text(0.0, 1.01, text.upper(), color=p["ink"], fontsize=13, fontweight="bold", va="bottom", ha="left",
             fontfamily="Roboto Condensed" if have else None,
             transform=offset_copy(fig.transFigure, fig=fig, x=15, y=0, units="points"))
    if meta:
        fig.text(1.0, 1.01, meta, ha="right", va="bottom", fontsize=8, color=p["muted"],
                 fontfamily="Roboto Mono" if have else None)


def empty(a, text):
    a.text(0.5, 0.5, text, transform=a.transAxes, ha="center", va="center", color=colors()["muted"], fontsize=9)
    a.set_xticks([])
    a.set_yticks([])


def save(fig, path):
    path = Path(path)
    if ax_style is not None:
        ax_style.save(fig, str(path), dpi=DPI)
    else:
        fig.savefig(path, dpi=DPI)
    plt.close(fig)
    return path

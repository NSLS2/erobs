"""Shared colors, fonts and Qt styles for the GUI.

Style references: aminechraibi/pyqt6-ui-designer and Leonxlnx/taste-skill.
"""

from __future__ import annotations

from PyQt6.QtGui import QColor, QFont, QFontDatabase, QPalette
from PyQt6.QtWidgets import QApplication, QWidget

# Style constants

# Surfaces
BG = "#0B0E14"               # window background
SURFACE = "#11151D"          # panels and splitters
SURFACE_HIGH = "#1A2030"     # hovered rows, secondary buttons
SURFACE_HIGHEST = "#222A3C"  # pressed / active emphasis
ELEVATED = "#161B26"         # popovers, menus, dropdowns

# Borders
OUTLINE = "#1E2533"          # dividers
OUTLINE_STRONG = "#2A3346"   # input borders

# Text
ON_SURFACE = "#E6EAF2"       # primary text
ON_SURFACE_MUTED = "#A0A8BC" # secondary / labels
ON_SURFACE_DIM = "#6B7385"   # placeholders and timestamps
DISABLED = "#5A6378"

# Accent
PRIMARY = "#5B8DEF"
PRIMARY_HOVER = "#7AA4F4"
PRIMARY_PRESSED = "#3F70D8"
PRIMARY_TINT = "#1F2C45"     # selected backgrounds
ON_PRIMARY = "#0B0F18"

# Status colors
SUCCESS = "#3DD68C"          # cached plan / connected
SUCCESS_DIM = "#1F4A35"
WARNING = "#F2B33D"          # dry-run / paused / waiting
WARNING_DIM = "#4A3A18"
DANGER = "#F26B6B"           # error / disconnected / aborted
DANGER_DIM = "#4A1F1F"

# Fonts
FONT_BODY = "Inter, 'Segoe UI', 'Helvetica Neue', Arial, sans-serif"

# Corner radii
R_SM = 4
R_MD = 6
R_PILL = 999


# Widget styles

DARK_QSS = f"""
/* Base */
QWidget {{
    background-color: {BG};
    color: {ON_SURFACE};
    font-family: {FONT_BODY};
    font-size: 13px;
}}

QMainWindow, QDialog {{
    background-color: {BG};
}}

QToolTip {{
    background-color: {ELEVATED};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE_STRONG};
    border-radius: {R_SM}px;
    padding: 6px 8px;
}}

/* Menus */
QMenuBar {{
    background-color: {BG};
    color: {ON_SURFACE_MUTED};
    border-bottom: 1px solid {OUTLINE};
    padding: 2px 4px;
}}
QMenuBar::item {{
    background: transparent;
    padding: 6px 12px;
    border-radius: {R_SM}px;
    color: {ON_SURFACE_MUTED};
}}
QMenuBar::item:selected {{
    background-color: {SURFACE_HIGH};
    color: {ON_SURFACE};
}}
QMenu {{
    background-color: {ELEVATED};
    border: 1px solid {OUTLINE_STRONG};
    border-radius: {R_MD}px;
    padding: 4px;
    color: {ON_SURFACE};
}}
QMenu::item {{
    padding: 6px 16px;
    border-radius: {R_SM}px;
}}
QMenu::item:selected {{
    background-color: {PRIMARY_TINT};
    color: {ON_SURFACE};
}}
QMenu::separator {{
    height: 1px;
    background: {OUTLINE};
    margin: 4px 6px;
}}

/* Buttons */
QPushButton {{
    background-color: {SURFACE_HIGH};
    color: {ON_SURFACE};
    border: 1px solid transparent;
    border-radius: {R_SM}px;
    padding: 6px 14px;
    font-weight: 500;
    min-height: 22px;
}}
QPushButton:hover {{
    background-color: {SURFACE_HIGHEST};
}}
QPushButton:pressed {{
    background-color: {SURFACE};
}}
QPushButton:focus {{
    border: 1px solid {PRIMARY};
    outline: none;
}}
QPushButton:disabled {{
    background-color: {SURFACE};
    color: {DISABLED};
    border-color: transparent;
}}
QPushButton:flat {{
    background: transparent;
    border: 1px solid transparent;
    color: {ON_SURFACE_MUTED};
}}
QPushButton:flat:hover {{
    background-color: {SURFACE_HIGH};
    color: {ON_SURFACE};
}}

/* Select button variants with setProperty("class", "primary") or "danger". */
QPushButton[class="primary"] {{
    background-color: {PRIMARY};
    color: {ON_PRIMARY};
    border: 1px solid {PRIMARY};
    font-weight: 600;
}}
QPushButton[class="primary"]:hover {{
    background-color: {PRIMARY_HOVER};
    border-color: {PRIMARY_HOVER};
}}
QPushButton[class="primary"]:pressed {{
    background-color: {PRIMARY_PRESSED};
}}
QPushButton[class="primary"]:disabled {{
    background-color: {SURFACE};
    color: {DISABLED};
    border-color: transparent;
}}
QPushButton[class="danger"] {{
    background-color: transparent;
    color: {DANGER};
    border: 1px solid transparent;
}}
QPushButton[class="danger"]:hover {{
    background-color: {DANGER_DIM};
    color: {DANGER};
}}
QPushButton[class="danger"]:disabled {{
    color: {DISABLED};
    background: transparent;
}}

/* Run controls: the frame owns the outer border; buttons own the dividers. */
QFrame#runBar {{
    background-color: {SURFACE_HIGH};
    border: 1px solid {OUTLINE};
    border-radius: {R_SM}px;
}}
QFrame#runBar > QPushButton,
QFrame#runBar > QPushButton[class="primary"],
QFrame#runBar > QPushButton[class="danger"] {{
    border: none;
    border-right: 1px solid {OUTLINE};
    border-top-left-radius: 0;
    border-top-right-radius: 0;
    border-bottom-left-radius: 0;
    border-bottom-right-radius: 0;
    margin: 0;
    padding: 6px 14px;
}}
QFrame#runBar > QPushButton#segLast,
QFrame#runBar > QPushButton[class="danger"]#segLast {{
    border-right: none;
}}
QFrame#runBar > QPushButton#segFirst,
QFrame#runBar > QPushButton[class="primary"]#segFirst {{
    border-top-left-radius: {R_SM}px;
    border-bottom-left-radius: {R_SM}px;
}}
QFrame#runBar > QPushButton#segLast {{
    border-top-right-radius: {R_SM}px;
    border-bottom-right-radius: {R_SM}px;
}}

/* Inputs */
QLineEdit, QTextEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox {{
    background-color: {BG};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE};
    border-radius: {R_SM}px;
    padding: 5px 10px;
    selection-background-color: {PRIMARY};
    selection-color: {ON_PRIMARY};
}}
QLineEdit:hover, QTextEdit:hover, QPlainTextEdit:hover,
QSpinBox:hover, QDoubleSpinBox:hover {{
    border-color: {OUTLINE_STRONG};
}}
QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus,
QSpinBox:focus, QDoubleSpinBox:focus {{
    border: 1px solid {PRIMARY};
    background-color: {BG};
}}
QLineEdit:disabled, QTextEdit:disabled, QPlainTextEdit:disabled {{
    background-color: {SURFACE};
    color: {ON_SURFACE_MUTED};
    border-color: {OUTLINE};
}}

/* Combo boxes */
QComboBox {{
    background-color: {BG};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE};
    border-radius: {R_SM}px;
    padding: 5px 10px;
    min-height: 22px;
}}
QComboBox:hover {{
    border-color: {OUTLINE_STRONG};
}}
QComboBox:focus {{
    border-color: {PRIMARY};
}}
QComboBox::drop-down {{
    width: 22px;
    border: none;
    background: transparent;
}}
QComboBox::down-arrow {{
    image: none;
    width: 0; height: 0;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid {ON_SURFACE_MUTED};
    margin-right: 8px;
}}
QComboBox QAbstractItemView {{
    background-color: {ELEVATED};
    border: 1px solid {OUTLINE_STRONG};
    border-radius: {R_MD}px;
    padding: 4px;
    outline: none;
    selection-background-color: {PRIMARY_TINT};
    selection-color: {ON_SURFACE};
}}

/* Checkboxes and radio buttons */
QCheckBox, QRadioButton {{
    color: {ON_SURFACE};
    spacing: 8px;
    padding: 4px 0;
}}
QCheckBox:disabled, QRadioButton:disabled {{
    color: {DISABLED};
}}
QCheckBox::indicator, QRadioButton::indicator {{
    width: 16px; height: 16px;
    border: 1px solid {OUTLINE_STRONG};
    background-color: {SURFACE};
}}
QCheckBox::indicator {{
    border-radius: 3px;
}}
QRadioButton::indicator {{
    border-radius: 8px;
}}
QCheckBox::indicator:hover, QRadioButton::indicator:hover {{
    border-color: {PRIMARY};
}}
QCheckBox::indicator:checked, QRadioButton::indicator:checked {{
    background-color: {PRIMARY};
    border-color: {PRIMARY};
}}

/* Group boxes */
QGroupBox {{
    background: transparent;
    border: none;
    border-top: 1px solid {OUTLINE};
    border-radius: 0;
    margin-top: 22px;
    padding: 14px 4px 4px 4px;
    font-weight: 500;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 0;
    top: 2px;
    padding: 0 0 6px 0;
    color: {ON_SURFACE_MUTED};
    background: transparent;
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 1px;
    text-transform: uppercase;
}}

/* Tabs */
QTabWidget::pane {{
    background: transparent;
    border: none;
    border-top: 1px solid {OUTLINE};
    top: 0;
}}
QTabBar {{
    background: transparent;
    qproperty-drawBase: 0;
}}
QTabBar::tab {{
    background: transparent;
    color: {ON_SURFACE_MUTED};
    padding: 8px 14px;
    border: none;
    border-bottom: 2px solid transparent;
    margin-right: 4px;
    font-weight: 500;
}}
QTabBar::tab:hover {{
    color: {ON_SURFACE};
}}
QTabBar::tab:selected {{
    background: transparent;
    color: {ON_SURFACE};
    border-bottom: 2px solid {PRIMARY};
}}

/* Lists and trees */
QListWidget, QTreeWidget, QTreeView, QListView {{
    background: transparent;
    color: {ON_SURFACE};
    border: none;
    outline: none;
    padding: 4px;
}}
QListWidget::item, QTreeWidget::item {{
    padding: 8px 12px;
    border-radius: {R_SM}px;
    margin: 1px 4px;
}}
QListWidget::item:hover, QTreeWidget::item:hover {{
    background-color: {SURFACE_HIGH};
    color: {ON_SURFACE};
}}
QListWidget::item:selected, QTreeWidget::item:selected {{
    background-color: {PRIMARY_TINT};
    color: {ON_SURFACE};
}}
QListWidget::item:selected:!active, QTreeWidget::item:selected:!active {{
    background-color: {SURFACE_HIGH};
    color: {ON_SURFACE};
}}

/* Splitter handles */
QSplitter::handle {{
    background-color: {OUTLINE};
}}
QSplitter::handle:horizontal {{
    width: 1px;
}}
QSplitter::handle:vertical {{
    height: 1px;
}}
QSplitter::handle:hover {{
    background-color: {PRIMARY};
}}

/* Progress bars */
QProgressBar {{
    background-color: {SURFACE};
    color: {ON_SURFACE};
    border: 1px solid {OUTLINE};
    border-radius: {R_SM}px;
    text-align: center;
    font-size: 11px;
    height: 16px;
}}
QProgressBar::chunk {{
    background-color: {PRIMARY};
    border-radius: 3px;
    margin: 1px;
}}

/* Scrollbars */
QScrollBar:vertical {{
    background: transparent;
    width: 10px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background: {OUTLINE_STRONG};
    border-radius: 4px;
    min-height: 28px;
    margin: 2px;
}}
QScrollBar::handle:vertical:hover {{
    background: {ON_SURFACE_DIM};
}}
QScrollBar:horizontal {{
    background: transparent;
    height: 10px;
    margin: 0;
}}
QScrollBar::handle:horizontal {{
    background: {OUTLINE_STRONG};
    border-radius: 4px;
    min-width: 28px;
    margin: 2px;
}}
QScrollBar::handle:horizontal:hover {{
    background: {ON_SURFACE_DIM};
}}
QScrollBar::add-line, QScrollBar::sub-line {{
    width: 0; height: 0;
    background: transparent;
}}
QScrollBar::add-page, QScrollBar::sub-page {{
    background: transparent;
}}

/* Frames and labels */
QFrame {{
    background: transparent;
}}
QLabel {{
    background: transparent;
    color: {ON_SURFACE};
}}

/* Table and tree headers */
QHeaderView::section {{
    background-color: {SURFACE};
    color: {ON_SURFACE_MUTED};
    border: none;
    border-bottom: 1px solid {OUTLINE};
    padding: 6px 10px;
    font-size: 11px;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}}

/* Status labels */
QLabel[status="success"] {{
    color: {SUCCESS};
    background-color: {SUCCESS_DIM};
    border: 1px solid {SUCCESS};
    border-radius: {R_PILL}px;
    padding: 2px 10px;
    font-size: 11px;
    font-weight: 600;
}}
QLabel[status="warning"] {{
    color: {WARNING};
    background-color: {WARNING_DIM};
    border: 1px solid {WARNING};
    border-radius: {R_PILL}px;
    padding: 2px 10px;
    font-size: 11px;
    font-weight: 600;
}}
QLabel[role="section"] {{
    color: {ON_SURFACE_MUTED};
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 1px;
    text-transform: uppercase;
    padding: 4px 0;
}}
QLabel[role="hint"] {{
    color: {ON_SURFACE_DIM};
    font-size: 11px;
}}
"""


def _palette() -> QPalette:
    """Match palette-based rendering and disabled colors to the stylesheet."""
    p = QPalette()
    p.setColor(QPalette.ColorRole.Window, QColor(BG))
    p.setColor(QPalette.ColorRole.WindowText, QColor(ON_SURFACE))
    p.setColor(QPalette.ColorRole.Base, QColor(SURFACE))
    p.setColor(QPalette.ColorRole.AlternateBase, QColor(SURFACE))
    p.setColor(QPalette.ColorRole.ToolTipBase, QColor(ELEVATED))
    p.setColor(QPalette.ColorRole.ToolTipText, QColor(ON_SURFACE))
    p.setColor(QPalette.ColorRole.Text, QColor(ON_SURFACE))
    p.setColor(QPalette.ColorRole.Button, QColor(SURFACE_HIGH))
    p.setColor(QPalette.ColorRole.ButtonText, QColor(ON_SURFACE))
    p.setColor(QPalette.ColorRole.BrightText, QColor(DANGER))
    p.setColor(QPalette.ColorRole.Link, QColor(PRIMARY))
    p.setColor(QPalette.ColorRole.Highlight, QColor(PRIMARY))
    p.setColor(QPalette.ColorRole.HighlightedText, QColor(ON_PRIMARY))
    p.setColor(QPalette.ColorRole.PlaceholderText, QColor(ON_SURFACE_DIM))
    for role in (QPalette.ColorRole.Text, QPalette.ColorRole.ButtonText,
                 QPalette.ColorRole.WindowText):
        p.setColor(QPalette.ColorGroup.Disabled, role, QColor(DISABLED))
    return p


def _load_fonts() -> str:
    """Find or load Inter, falling back to the system sans-serif font."""
    families = set(QFontDatabase.families())
    for cand in ("Inter Variable", "Inter"):
        if cand in families:
            return cand
    # Debian/Ubuntu package paths, used when Qt has not discovered Inter.
    for path in (
        "/usr/share/fonts/truetype/inter-vf/InterVariable.ttf",
        "/usr/share/fonts/truetype/inter/Inter-Regular.otf",
    ):
        idx = QFontDatabase.addApplicationFont(path)
        if idx >= 0:
            fams = QFontDatabase.applicationFontFamilies(idx)
            if fams:
                return fams[0]
    return "Sans Serif"


def icon(name: str, color: str = ON_SURFACE):
    """Return a qtawesome icon, or an empty icon if loading fails."""
    try:
        import qtawesome as qta  # type: ignore
        return qta.icon(name, color=color)
    except Exception:
        from PyQt6.QtGui import QIcon
        return QIcon()


def apply(app: QApplication) -> None:
    """Apply the dark palette, stylesheet and application font."""
    app.setStyle("Fusion")  # consistent baseline across platforms
    family = _load_fonts()
    f = QFont(family)
    f.setPointSize(10)
    f.setHintingPreference(QFont.HintingPreference.PreferFullHinting)
    app.setFont(f)
    app.setPalette(_palette())
    app.setStyleSheet(DARK_QSS)


def toggle_dark_mode(app: QApplication, enabled: bool) -> None:
    """Apply the dark theme or restore the default palette and stylesheet."""
    if enabled:
        apply(app)
    else:
        app.setPalette(app.style().standardPalette())
        app.setStyleSheet("")


def restyle(widget: QWidget) -> None:
    """Refresh widget styling after changing a dynamic property."""
    widget.style().unpolish(widget)
    widget.style().polish(widget)
    widget.update()

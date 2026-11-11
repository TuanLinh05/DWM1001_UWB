"""Design Tokens & Light/Dark QSS Theme System for UWB Fast Studio.

Follows the PyQt6 Modern Enterprise Design System:
- 4px base spacing grid (SPACING_UNIT = 4)
- Explicit surface hierarchy for both Dark Mode and Light Mode (zero transparent/black artifacts)
- High-contrast typography & tabular monospace numerals
"""

from __future__ import annotations

from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QApplication

# ─── 1. Light Mode Palette (Clean Technical Daylight) ────────────────────────
COLOR_BACKGROUND           = "#F1F5F9"  # Slate 100 main workspace background
COLOR_SURFACE_LOWEST       = "#FFFFFF"  # Pure white cards & inputs
COLOR_SURFACE_LOW          = "#F8FAFC"  # Slate 50 alternating rows / subcards
COLOR_SURFACE              = "#E2E8F0"  # Slate 200 sidebar & header surface
COLOR_SURFACE_HIGH         = "#CBD5E1"  # Slate 300 hover / active states
COLOR_SURFACE_HIGHEST      = "#94A3B8"

COLOR_ON_SURFACE           = "#0F172A"  # Slate 900 primary text
COLOR_ON_SURFACE_VARIANT   = "#475569"  # Slate 600 secondary label text
COLOR_OUTLINE              = "#64748B"  # Slate 500 strong border
COLOR_OUTLINE_VARIANT      = "#CBD5E1"  # Slate 300 subtle card/table border

COLOR_PRIMARY              = "#0284C7"  # Sky 600 primary action
COLOR_ON_PRIMARY           = "#FFFFFF"
COLOR_PRIMARY_CONTAINER    = "#0369A1"  # Sky 700 hover
COLOR_PRIMARY_FIXED        = "#E0F2FE"  # Sky 100 active nav highlight

# ─── 2. Dark Mode Palette (Deep Carbon Telemetry Studio) ─────────────────────
DARK_BACKGROUND            = "#0B1120"  # Deep navy-carbon workspace background
DARK_SURFACE               = "#111827"  # Card & topbar surface
DARK_SURFACE_CONTAINER     = "#1E293B"  # Subcard / table header / hover surface
DARK_SURFACE_HIGH          = "#334155"  # Pressed / active surface
DARK_BORDER                = "#1E293B"  # Subtle card border
DARK_OUTLINE               = "#475569"  # Input / button border
DARK_ON_SURFACE            = "#F8FAFC"  # Crisp white-slate text
DARK_ON_SURFACE_VARIANT    = "#94A3B8"  # Muted telemetry label text
DARK_PRIMARY               = "#0EA5E9"  # Sky 500 primary button
DARK_PRIMARY_CONTAINER     = "#0284C7"  # Sky 600 hover
DARK_PRIMARY_FIXED         = "#0C4A6E"  # Sky 900 active nav pill

# ─── 3. Semantic & Anchor Colors ─────────────────────────────────────────────
COLOR_SUCCESS              = "#10B981"
COLOR_SUCCESS_DARK_TEXT    = "#065F46"
COLOR_SUCCESS_BG           = "#D1FAE5"

COLOR_WARNING              = "#F59E0B"
COLOR_WARNING_DARK_TEXT    = "#92400E"
COLOR_WARNING_BG           = "#FEF3C7"

COLOR_DANGER               = "#EF4444"
COLOR_DANGER_DARK_TEXT     = "#991B1B"
COLOR_DANGER_BG            = "#FEE2E2"

COLOR_INFO_ACCENT          = "#0284C7"
COLOR_INFO_BG              = "#E0F2FE"

ANCHOR_PALETTE = (
    "#2563EB",  # A1 Blue
    "#7C3AED",  # A2 Violet
    "#0891B2",  # A3 Cyan
    "#EA580C",  # A4 Orange
    "#DB2777",  # A5 Pink
    "#059669",  # A6 Emerald
    "#DC2626",  # A7 Red
    "#4F46E5",  # A8 Indigo
)
ANCHOR_HEX = ANCHOR_PALETTE

# ─── 4. Typography & 4px Grid Tokens ─────────────────────────────────────────
FONT_HEADING  = "Segoe UI, Inter, sans-serif"
FONT_BODY     = "Segoe UI, Inter, sans-serif"
FONT_MONO     = "Consolas, Cascadia Mono, Fira Code, monospace"

SPACING_UNIT = 4
SPACING_XS   = 4
SPACING_SM   = 8
SPACING_MD   = 12
SPACING_LG   = 16
SPACING_XL   = 24

RADIUS_SM      = 4
RADIUS_DEFAULT = 6
RADIUS_MD      = 8
RADIUS_LG      = 10


def _build_qss(
    *,
    is_dark: bool,
    bg: str,
    surface_card: str,
    surface_sub: str,
    surface_bar: str,
    surface_hover: str,
    on_surface: str,
    on_surface_muted: str,
    border_subtle: str,
    border_strong: str,
    primary: str,
    primary_hover: str,
    primary_soft_bg: str,
    primary_soft_fg: str,
) -> str:
    badge_ok_bg = "#064E3B" if is_dark else "#D1FAE5"
    badge_ok_fg = "#34D399" if is_dark else "#065F46"

    badge_info_bg = "#0C4A6E" if is_dark else "#E0F2FE"
    badge_info_fg = "#38BDF8" if is_dark else "#0369A1"

    badge_err_bg = "#7F1D1D" if is_dark else "#FEE2E2"
    badge_err_fg = "#FCA5A5" if is_dark else "#991B1B"

    badge_neutral_bg = "#1E293B" if is_dark else "#E2E8F0"
    badge_neutral_fg = "#94A3B8" if is_dark else "#475569"

    return f"""
/* ─── Global Base ─────────────────────────────────────────── */
QWidget {{
    font-family: "Segoe UI", "Inter", sans-serif;
    font-size: 12px;
    color: {on_surface};
    outline: none;
}}

QMainWindow, QDialog, QStackedWidget, QWidget#workspace_page, QWidget#central_root {{
    background-color: {bg};
}}

QLabel, QCheckBox {{
    background-color: transparent;
    color: {on_surface};
}}

QCheckBox {{
    spacing: 6px;
    font-weight: 600;
}}

QCheckBox::indicator {{
    width: 15px;
    height: 15px;
    border-radius: 3px;
    border: 1px solid {border_strong};
    background-color: {surface_sub};
}}

QCheckBox::indicator:checked {{
    background-color: {primary};
    border: 1px solid {primary};
}}

QSplitter::handle {{
    background-color: {border_subtle};
}}
QSplitter::handle:horizontal {{
    width: 4px;
}}
QSplitter::handle:vertical {{
    height: 4px;
}}

/* ─── Top Bar & Left Sidebar ──────────────────────────────── */
QFrame#top_bar, QFrame#topBar {{
    background-color: {surface_bar};
    border-bottom: 1px solid {border_subtle};
}}

QFrame#sidebar {{
    background-color: {surface_bar};
    border-right: 1px solid {border_subtle};
}}

QLabel#brand_title {{
    font-size: 14px;
    font-weight: 800;
    letter-spacing: 0.5px;
    color: {primary};
}}

/* ─── Sidebar Navigation Buttons ──────────────────────────── */
QPushButton#nav_btn {{
    background-color: transparent;
    border: 1px solid transparent;
    border-left: 3px solid transparent;
    border-radius: {RADIUS_DEFAULT}px;
    padding: 8px 12px;
    text-align: left;
    color: {on_surface_muted};
    font-size: 12px;
    font-weight: 600;
    min-height: 22px;
}}

QPushButton#nav_btn:hover {{
    background-color: {surface_hover};
    color: {on_surface};
}}

QPushButton#nav_btn:checked {{
    background-color: {primary_soft_bg};
    color: {primary_soft_fg};
    border-left: 3px solid {primary};
    font-weight: 700;
}}

/* ─── Cards & Containers ──────────────────────────────────── */
QFrame#panel_card, QFrame[class="card"] {{
    background-color: {surface_card};
    border: 1px solid {border_subtle};
    border-radius: {RADIUS_LG}px;
}}

QFrame#stat_card, QFrame#sub_card, QFrame[class="subcard"] {{
    background-color: {surface_card};
    border: 1px solid {border_subtle};
    border-radius: {RADIUS_MD}px;
}}

/* ─── Card Typography ─────────────────────────────────────── */
QLabel#section_header {{
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.4px;
    color: {on_surface_muted};
}}

QLabel#card_title {{
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.3px;
    color: {on_surface_muted};
}}

QLabel#metric_value {{
    font-family: "Consolas", "Cascadia Mono", monospace;
    font-size: 16px;
    font-weight: 700;
    color: {on_surface};
}}

QLabel#metric_sub {{
    font-size: 11px;
    color: {on_surface_muted};
}}

/* ─── Connection & Status Badges ──────────────────────────── */
QLabel#badge_ok {{
    background-color: {badge_ok_bg};
    color: {badge_ok_fg};
    border-radius: {RADIUS_SM}px;
    padding: 3px 8px;
    font-size: 11px;
    font-weight: 700;
}}

QLabel#badge_info {{
    background-color: {badge_info_bg};
    color: {badge_info_fg};
    border-radius: {RADIUS_SM}px;
    padding: 3px 8px;
    font-size: 11px;
    font-weight: 700;
}}

QLabel#badge_err {{
    background-color: {badge_err_bg};
    color: {badge_err_fg};
    border-radius: {RADIUS_SM}px;
    padding: 3px 8px;
    font-size: 11px;
    font-weight: 700;
}}

QLabel#badge_neutral {{
    background-color: {badge_neutral_bg};
    color: {badge_neutral_fg};
    border-radius: {RADIUS_SM}px;
    padding: 3px 8px;
    font-size: 11px;
    font-weight: 700;
}}

/* ─── Buttons ─────────────────────────────────────────────── */
QPushButton {{
    background-color: {surface_sub};
    color: {on_surface};
    border: 1px solid {border_strong};
    border-radius: {RADIUS_DEFAULT}px;
    padding: 5px 10px;
    font-size: 12px;
    font-weight: 600;
    min-height: 20px;
}}

QPushButton:hover {{
    background-color: {surface_hover};
    border-color: {primary};
}}

QPushButton:checked {{
    background-color: {primary_soft_bg};
    color: {primary_soft_fg};
    border: 1px solid {primary};
}}

QPushButton:disabled {{
    color: {on_surface_muted};
    background-color: {surface_bar};
    border-color: {border_subtle};
}}

QPushButton#primary_btn {{
    background-color: {primary};
    color: #FFFFFF;
    border: 1px solid {primary};
}}

QPushButton#primary_btn:hover {{
    background-color: {primary_hover};
}}

QPushButton#success_btn {{
    background-color: #059669;
    color: #FFFFFF;
    border: 1px solid #059669;
}}

QPushButton#success_btn:hover {{
    background-color: #047857;
}}

QPushButton#danger_btn {{
    background-color: #DC2626;
    color: #FFFFFF;
    border: 1px solid #DC2626;
}}

QPushButton#danger_btn:hover {{
    background-color: #B91C1C;
}}

/* ─── Inputs & Combos ─────────────────────────────────────── */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background-color: {surface_sub};
    color: {on_surface};
    border: 1px solid {border_strong};
    border-radius: {RADIUS_DEFAULT}px;
    padding: 3px 8px;
    font-size: 12px;
    min-height: 22px;
}}

QAbstractSpinBox QLineEdit {{
    background-color: transparent;
    color: {on_surface};
    border: none;
    padding: 0px;
    margin: 0px;
    min-height: 0px;
}}

QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
    border: 1px solid {primary};
}}

QComboBox::drop-down {{
    border: none;
    width: 20px;
}}

QComboBox QAbstractItemView {{
    background-color: {surface_card};
    color: {on_surface};
    border: 1px solid {border_strong};
    selection-background-color: {primary_soft_bg};
    selection-color: {primary_soft_fg};
}}

/* ─── Tables & Trees ──────────────────────────────────────── */
QTableWidget, QTreeWidget, QTreeView {{
    background-color: {surface_card};
    alternate-background-color: {surface_sub};
    color: {on_surface};
    border: 1px solid {border_subtle};
    border-radius: {RADIUS_DEFAULT}px;
    gridline-color: {border_subtle};
    selection-background-color: {primary_soft_bg};
    selection-color: {primary_soft_fg};
    font-size: 12px;
}}

QTableWidget::item {{
    padding: 2px 6px;
}}

QTreeWidget::item, QTreeView::item {{
    padding: 2px 6px;
    color: {on_surface};
}}

QTreeWidget::item:hover, QTreeView::item:hover {{
    background-color: {surface_hover};
}}

QTreeWidget::item:selected, QTreeView::item:selected {{
    background-color: {primary_soft_bg};
    color: {primary_soft_fg};
}}

QHeaderView {{
    background-color: {surface_sub};
    border: none;
}}

QHeaderView::section {{
    background-color: {surface_sub};
    color: {on_surface_muted};
    font-size: 11px;
    font-weight: 700;
    padding: 5px 6px;
    border: none;
    border-right: 1px solid {border_subtle};
    border-bottom: 1px solid {border_subtle};
}}

/* ─── Tabs & ScrollAreas ──────────────────────────────────── */
QTabWidget::pane {{
    border: 1px solid {border_subtle};
    border-radius: {RADIUS_DEFAULT}px;
    background-color: {surface_card};
    top: -1px;
}}

QTabBar::tab {{
    background-color: {surface_sub};
    color: {on_surface_muted};
    border: 1px solid {border_subtle};
    border-bottom: none;
    border-top-left-radius: {RADIUS_SM}px;
    border-top-right-radius: {RADIUS_SM}px;
    padding: 5px 12px;
    margin-right: 2px;
    font-size: 11px;
    font-weight: 700;
}}

QTabBar::tab:selected {{
    background-color: {surface_card};
    color: {primary};
    border-bottom: 2px solid {primary};
}}

QTabBar::tab:hover:!selected {{
    background-color: {surface_hover};
    color: {on_surface};
}}

QScrollArea {{
    background-color: transparent;
    border: none;
}}

/* ─── PlainTextEdit & ProgressBar ─────────────────────────── */
QPlainTextEdit {{
    background-color: {surface_sub};
    color: {on_surface};
    border: 1px solid {border_subtle};
    border-radius: {RADIUS_DEFAULT}px;
    font-family: "Consolas", "Cascadia Mono", monospace;
    font-size: 11px;
    padding: 6px;
}}

QProgressBar {{
    background-color: {surface_sub};
    border: 1px solid {border_subtle};
    border-radius: {RADIUS_SM}px;
    min-height: 12px;
    max-height: 12px;
    text-align: center;
    font-size: 10px;
    color: {on_surface};
}}

QProgressBar::chunk {{
    background-color: {primary};
    border-radius: {RADIUS_SM}px;
}}
"""


LIGHT_QSS = _build_qss(
    is_dark=False,
    bg=COLOR_BACKGROUND,
    surface_card=COLOR_SURFACE_LOWEST,
    surface_sub=COLOR_SURFACE_LOW,
    surface_bar=COLOR_SURFACE_LOWEST,
    surface_hover=COLOR_SURFACE,
    on_surface=COLOR_ON_SURFACE,
    on_surface_muted=COLOR_ON_SURFACE_VARIANT,
    border_subtle=COLOR_OUTLINE_VARIANT,
    border_strong=COLOR_OUTLINE,
    primary=COLOR_PRIMARY,
    primary_hover=COLOR_PRIMARY_CONTAINER,
    primary_soft_bg=COLOR_PRIMARY_FIXED,
    primary_soft_fg="#0369A1",
)

DARK_QSS = _build_qss(
    is_dark=True,
    bg=DARK_BACKGROUND,
    surface_card=DARK_SURFACE,
    surface_sub=DARK_SURFACE_CONTAINER,
    surface_bar=DARK_SURFACE,
    surface_hover=DARK_SURFACE_HIGH,
    on_surface=DARK_ON_SURFACE,
    on_surface_muted=DARK_ON_SURFACE_VARIANT,
    border_subtle=DARK_BORDER,
    border_strong=DARK_OUTLINE,
    primary=DARK_PRIMARY,
    primary_hover=DARK_PRIMARY_CONTAINER,
    primary_soft_bg=DARK_PRIMARY_FIXED,
    primary_soft_fg="#38BDF8",
)


class ThemeManager(QObject):
    """Manages light and dark QSS stylesheets across the application."""

    theme_changed = pyqtSignal(bool)

    def __init__(self, app: QApplication) -> None:
        super().__init__()
        self.app = app
        self._dark = False

    @property
    def is_dark(self) -> bool:
        return self._dark

    def apply_light(self) -> bool:
        self._dark = False
        self.app.setStyleSheet(LIGHT_QSS)
        self.theme_changed.emit(False)
        return False

    def apply_dark(self) -> bool:
        self._dark = True
        self.app.setStyleSheet(DARK_QSS)
        self.theme_changed.emit(True)
        return True

    def toggle(self) -> bool:
        if self._dark:
            return self.apply_light()
        return self.apply_dark()

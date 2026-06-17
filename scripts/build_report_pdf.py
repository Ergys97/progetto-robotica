#!/usr/bin/env python3
"""Build the final report PDF without external document-conversion tools."""

from __future__ import annotations

import textwrap
from pathlib import Path

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs" / "report" / "relazione_finale.pdf"
FIG_DIR = ROOT / "docs" / "report" / "figures"
ARCHITECTURE_IMAGE = ROOT / "architettura_ros2.png"

PAGE_W = 8.27
PAGE_H = 11.69
MARGIN_CM = 2.0
MARGIN_X = MARGIN_CM / 2.54
MARGIN_Y = MARGIN_CM / 2.54
CONTENT_W = PAGE_W - 2 * MARGIN_X
TOP_Y = PAGE_H - MARGIN_Y
BODY_FONT_SIZE = 11.0
LINE_SPACING = 1.08
BODY_LEADING = BODY_FONT_SIZE * LINE_SPACING / 72.0
CAPTION_FONT_SIZE = 9.0
CAPTION_LEADING = CAPTION_FONT_SIZE * LINE_SPACING / 72.0
TABLE_FONT_SIZE = 8.2

TOKENS = {
    "surface": "#FFFFFF",
    "ink": "#1F2430",
    "muted": "#5F667A",
    "rule": "#D7DBE7",
    "blue": "#2E4780",
    "blue_light": "#EAF1FE",
    "gold_light": "#FFF4C2",
}


def new_page(pdf: PdfPages) -> tuple[plt.Figure, plt.Axes]:
    fig = plt.figure(figsize=(PAGE_W, PAGE_H), facecolor=TOKENS["surface"])
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, PAGE_W)
    ax.set_ylim(0, PAGE_H)
    ax.axis("off")
    return fig, ax


def finish_page(pdf: PdfPages, fig: plt.Figure, page: int) -> None:
    fig.text(0.5, 0.035, f"{page}", ha="center", va="center", fontsize=8, color=TOKENS["muted"])
    pdf.savefig(fig)
    plt.close(fig)


def add_heading(ax: plt.Axes, y: float, text: str, size: float = 14.0) -> float:
    ax.text(MARGIN_X, y, text, ha="left", va="top", fontsize=size, weight="bold", color=TOKENS["ink"])
    ax.plot([MARGIN_X, PAGE_W - MARGIN_X], [y - 0.18, y - 0.18], color=TOKENS["rule"], linewidth=0.8)
    return y - 0.34


def add_paragraph(ax: plt.Axes, y: float, text: str, width: int = 80, size: float = BODY_FONT_SIZE, leading: float = BODY_LEADING) -> float:
    lines: list[str] = []
    for paragraph in text.split("\n"):
        if not paragraph.strip():
            lines.append("")
        else:
            lines.extend(textwrap.wrap(paragraph.strip(), width=width, break_long_words=False))

    for line in lines:
        ax.text(MARGIN_X, y, line, ha="left", va="top", fontsize=size, color=TOKENS["ink"])
        y -= leading
    return y - 0.04


def add_bullets(ax: plt.Axes, y: float, items: list[str], width: int = 76, size: float = BODY_FONT_SIZE) -> float:
    for item in items:
        wrapped = textwrap.wrap(item, width=width, break_long_words=False)
        ax.text(MARGIN_X + 0.15, y, "-", ha="left", va="top", fontsize=size, color=TOKENS["blue"])
        ax.text(MARGIN_X + 0.36, y, wrapped[0], ha="left", va="top", fontsize=size, color=TOKENS["ink"])
        y -= BODY_LEADING
        for line in wrapped[1:]:
            ax.text(MARGIN_X + 0.36, y, line, ha="left", va="top", fontsize=size, color=TOKENS["ink"])
            y -= BODY_LEADING
    return y - 0.04


def add_image(ax: plt.Axes, y: float, image_name: str | Path, caption: str, height: float, width: float = CONTENT_W) -> float:
    image_path = image_name if isinstance(image_name, Path) else FIG_DIR / image_name
    image = mpimg.imread(image_path)
    x = (PAGE_W - width) / 2
    ax.imshow(image, extent=[x, x + width, y - height, y], aspect="auto")
    wrapped = textwrap.wrap(caption, width=86, break_long_words=False)
    y -= height + 0.12
    for line in wrapped:
        ax.text(MARGIN_X, y, line, ha="left", va="top", fontsize=CAPTION_FONT_SIZE, color=TOKENS["muted"], style="italic")
        y -= CAPTION_LEADING
    return y - 0.08


def add_table(ax: plt.Axes, y: float, rows: list[list[str]], col_widths: list[float], row_h: float = 0.31) -> float:
    x0 = MARGIN_X
    table_w = sum(col_widths)
    for r, row in enumerate(rows):
        x = x0
        fill = TOKENS["blue_light"] if r == 0 else ("#F8F9FC" if r % 2 == 0 else "#FFFFFF")
        ax.add_patch(plt.Rectangle((x0, y - row_h), table_w, row_h, facecolor=fill, edgecolor=TOKENS["rule"], linewidth=0.5))
        for c, value in enumerate(row):
            weight = "bold" if r == 0 else "normal"
            ax.text(x + 0.04, y - 0.08, value, ha="left", va="top", fontsize=TABLE_FONT_SIZE, weight=weight, color=TOKENS["ink"])
            x += col_widths[c]
            if c < len(row) - 1:
                ax.plot([x, x], [y - row_h, y], color=TOKENS["rule"], linewidth=0.4)
        y -= row_h
    return y - 0.14


def build() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with PdfPages(OUT) as pdf:
        fig, ax = new_page(pdf)
        ax.text(
            MARGIN_X,
            10.70,
            "Teleoperazione ROS 2 di Unitree G1\ncon dashboard realtime\ne validazione sperimentale",
            ha="left",
            va="top",
            fontsize=20,
            weight="bold",
            color=TOKENS["ink"],
            linespacing=1.18,
        )
        ax.text(MARGIN_X, 9.45, "Progetto di Robotica - Traccia 10", fontsize=12, color=TOKENS["blue"], weight="bold")
        ax.text(MARGIN_X, 9.10, "Studente: Ergys Perdeda", fontsize=10.5, color=TOKENS["ink"])
        ax.text(MARGIN_X, 8.86, "Anno accademico: 2025/2026", fontsize=10.5, color=TOKENS["ink"])
        ax.add_patch(plt.Rectangle((MARGIN_X, 8.10), CONTENT_W, 0.04, facecolor=TOKENS["blue"], edgecolor="none"))

        y = add_heading(ax, 7.52, "Abstract tecnico", size=14)
        y = add_paragraph(
            ax,
            y,
            "Il progetto realizza un prototipo ROS 2 per teleoperare il robot umanoide Unitree G1 "
            "in simulazione MuJoCo, monitorarlo tramite dashboard web realtime e validarne il "
            "comportamento con metriche sperimentali. Il sistema integra input tastiera/gamepad, "
            "pubblicazione dei topic di telemetria, rilevamento di condizioni critiche e "
            "registrazione delle sessioni tramite rosbag2. La validazione su piano regolare mostra "
            "frequenza telemetrica media pari a 33.51 Hz, latenza comando p95 pari a 23.45 ms e "
            "replay deterministico con MSE nullo nella prova selezionata. Lo scenario a ostacoli "
            "conferma la reattività degli alert, con 4 eventi di caduta, 79 flight phase e variazioni "
            "coerenti dei contatti ai piedi.",
        )
        y = add_heading(ax, y - 0.12, "Obiettivo", size=14)
        y = add_paragraph(
            ax,
            y,
            "L'obiettivo non è solo avviare una simulazione, ma costruire una architettura robotica "
            "integrata, osservabile e misurabile: il robot viene controllato tramite comandi di "
            "velocità, la telemetria viene pubblicata su topic ROS 2 e una dashboard web permette "
            "di monitorare in tempo reale lo stato del sistema.",
        )
        add_bullets(
            ax,
            y,
            [
                "teleoperazione tramite tastiera o gamepad",
                "simulazione fisica del robot Unitree G1 in MuJoCo",
                "dashboard realtime per IMU, giunti, odometria, contatti dei piedi e alert",
                "registrazione, replay e valutazione quantitativa delle sessioni sperimentali",
            ],
        )
        finish_page(pdf, fig, 1)

        fig, ax = new_page(pdf)
        y = add_heading(ax, TOP_Y, "Architettura del sistema", size=15)
        y = add_paragraph(
            ax,
            y,
            "L'architettura è basata su ROS 2 Jazzy e separa simulazione, interfaccia utente e "
            "analisi sperimentale. Il nodo mujoco_sim gestisce il modello MuJoCo e pubblica la "
            "telemetria; web_teleop integra server web, dashboard e input utente; replay_eval "
            "riproduce una sessione registrata e calcola l'errore di traiettoria.",
        )
        y = add_image(
            ax,
            y - 0.04,
            ARCHITECTURE_IMAGE,
            "Figura 1 - Architettura logica del sistema e principali flussi tra input, dashboard, simulazione, rosbag2 e replay.",
            height=3.43,
        )
        topic_rows = [
            ["Topic", "Ruolo"],
            ["/cmd_vel", "Comando velocità lineare e angolare verso il simulatore"],
            ["/imu, /joint_states, /odom", "Telemetria inerziale, giunti e traiettoria"],
            ["/contacts/left, /contacts/right", "Stato di contatto dei piedi"],
            ["/fall_detected", "Segnalazione di caduta basata su roll/pitch"],
            ["/metrics/cmd_latency_ms", "Latenza tra comando utente e ricezione nel simulatore"],
        ]
        add_table(ax, y, topic_rows, [2.05, 4.55], row_h=0.30)
        finish_page(pdf, fig, 2)

        fig, ax = new_page(pdf)
        y = add_heading(ax, TOP_Y, "Scenari e metriche", size=15)
        y = add_paragraph(
            ax,
            y,
            "Sono stati definiti due scenari: flat, usato per calibrare e misurare frequenza, "
            "latenza e replay; obstacle_course, con rampa e piccoli step, usato per validare "
            "contatti, flight phase e rilevamento caduta. La separazione evita di attribuire a "
            "un singolo test obiettivi sperimentali troppo diversi.",
        )
        metric_rows = [
            ["Metrica", "Target", "Motivazione"],
            ["Frequenza telemetria", ">= 30 Hz", "Aggiornamento fluido dashboard"],
            ["Latenza comando", "< 50 ms", "Teleoperazione reattiva"],
            ["MSE replay", "< 1e-4 m^2", "Riproducibilità traiettoria"],
            ["Fall/flight/contact", "eventi rilevati", "Validazione alert di sicurezza"],
        ]
        y = add_table(ax, y, metric_rows, [2.20, 1.45, 2.95], row_h=0.33)
        y = add_image(
            ax,
            y - 0.08,
            "latenza_comandi.png",
            "Figura 2 - Media e p95 restano sotto il target di 50 ms; il massimo obstacle è annotato come picco isolato.",
            height=3.85,
        )
        finish_page(pdf, fig, 3)

        fig, ax = new_page(pdf)
        y = add_heading(ax, TOP_Y, "Risultati sperimentali", size=15)
        result_rows = [
            ["Scenario", "Metrica", "Valore osservato", "Esito"],
            ["flat", "Telemetria media/min", "33.51 Hz / 32 Hz", "OK"],
            ["flat", "Latenza media/p95/max", "7.53 / 23.45 / 28.35 ms", "OK"],
            ["flat", "MSE replay", "0.00000000 m^2", "OK"],
            ["obstacle", "Telemetria media/min", "32.93 Hz / 3 Hz", "OK medio"],
            ["obstacle", "Latenza media/p95", "9.57 / 26.35 ms", "OK"],
            ["obstacle", "Fall / flight", "4 / 79 eventi", "OK"],
            ["obstacle", "Perdite contatto L/R", "114 / 139", "OK"],
        ]
        y = add_table(ax, y, result_rows, [1.22, 2.22, 2.35, 0.80], row_h=0.31)
        y = add_paragraph(
            ax,
            y,
            "Il replay deterministico è stato valutato sullo scenario flat, dove la traiettoria è "
            "più adatta a un confronto stabile. Lo scenario obstacle è invece orientato alla "
            "validazione degli alert e degli stati critici; la frequenza media resta sopra target "
            "ma il minimo transitorio a 3 Hz viene considerato un rallentamento locale.",
        )
        y = add_image(
            ax,
            y - 0.06,
            "traiettoria_flat.png",
            "Figura 3 - Traiettoria XY sul piano usata per la verifica del replay deterministico.",
            height=2.35,
            width=5.70,
        )
        y = add_image(
            ax,
            y - 0.02,
            "eventi_obstacle.png",
            "Figura 4 - Conteggio degli eventi critici nello scenario obstacle_course: caduta, flight phase e perdite di contatto.",
            height=2.55,
        )
        finish_page(pdf, fig, 4)

        fig, ax = new_page(pdf)
        y = add_heading(ax, TOP_Y, "Analisi critica e conclusioni", size=15)
        y = add_paragraph(
            ax,
            y,
            "Nel grafico seguente i segmenti temporali sono rappresentati in sequenza relativa, "
            "poiché durante la prova obstacle sono presenti reset della simulazione e discontinuità "
            "nel log. Questa scelta evita di sovrapporre campioni con lo stesso sim_time.",
        )
        y = add_image(
            ax,
            y,
            "assetto_quota_obstacle.png",
            "Figura 5 - Roll, pitch e quota base nello scenario a ostacoli; i segmenti sono separati nei punti di reset/discontinuita.",
            height=4.05,
        )
        y = add_paragraph(
            ax,
            y,
            "Il prototipo soddisfa gli obiettivi principali della traccia: integra simulazione "
            "MuJoCo, middleware ROS 2, teleoperazione, dashboard web e strumenti di registrazione "
            "e replay. La presenza di metriche quantitative consente di valutare il progetto non "
            "solo come demo visuale, ma come sistema sperimentale riproducibile.",
        )
        y = add_paragraph(
            ax,
            y,
            "Restano alcuni limiti: il progetto è interamente simulativo, la velocità comandata "
            "non è calibrata automaticamente rispetto alla velocità effettiva, e l'odometria non "
            "è stata confrontata in modo sistematico con il ground truth MuJoCo. Nello scenario "
            "a ostacoli restano inoltre picchi occasionali di latenza e brevi cali di frequenza. "
            "Sviluppi futuri includono calibrazione automatica, analisi del drift laterale e "
            "adattamento a robot fisico.",
        )
        finish_page(pdf, fig, 5)


if __name__ == "__main__":
    build()

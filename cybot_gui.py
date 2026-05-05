#!/usr/bin/env python3
"""
CyBot Cluster Detection – Control Panel GUI
CPRE 288 · Team SH-4

Supports two transports:
  • Serial — direct USB/COM (e.g. when CyBot is tethered for debugging)
  • TCP    — WiFi over the CyBot router (default 192.168.1.1:288)

Run:   python cybot_gui.py
Needs: pip install pyserial matplotlib numpy
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from typing import Optional
import serial
import serial.tools.list_ports
import socket
import threading
import queue
import re
import math
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as patches

# ── Color Palette ─────────────────────────────────────────────────────────────
BG_DARK   = "#1a1a2e"
BG_PANEL  = "#16213e"
BG_CARD   = "#0f3460"
FG_WHITE  = "#eaeaea"
FG_GREEN  = "#4ecca3"
FG_RED    = "#ff6b6b"
FG_YELLOW = "#ffd460"
FG_BLUE   = "#00b4d8"
FG_ORANGE = "#f4a261"
BTN_GREEN = "#2d6a4f"
BTN_RED   = "#9b2226"
BTN_BLUE  = "#1d3557"
BTN_GRAY  = "#3d3d5c"
BTN_PURP  = "#6b3fa0"
BTN_TEAL  = "#1d6091"


class CyBotGUI:

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("CyBot  ·  Invasive Fish Cluster Detector  ·  Team SH-4")
        self.root.geometry("1280x800")
        self.root.minsize(1020, 680)
        self.root.configure(bg=BG_DARK)

        # ── Connection state ──
        # Transport: "Serial" (USB/COM) or "TCP" (WiFi via CyBot router)
        self.transport:   str = "Serial"
        self.serial_conn: Optional[serial.Serial] = None
        self.tcp_sock:    Optional[socket.socket] = None
        self.rx_thread:   Optional[threading.Thread] = None
        self.rx_queue:    queue.Queue = queue.Queue()
        self.connected    = False
        self.mode         = "AUTO"

        # ── Parsed data ──
        self.scan_angles: list = []   # degrees
        self.scan_ping:   list = []   # cm (PING sensor)
        self.scan_ir:     list = []   # cm (IR calibration)
        self.objects:     list = []   # list of dicts
        self.clusters:    list = []   # list of dicts
        self.best_route:  Optional[dict] = None

        # Parsing section flags
        self._in_scan     = False
        self._in_objects  = False
        self._in_clusters = False

        self._build_ui()
        self._poll_queue()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self):
        """Clean shutdown: stop RX thread and close serial port before exit."""
        try:
            if self.connected:
                self._disconnect()
        except Exception:
            pass
        self.root.destroy()

    # ─────────────────────────────────────────────────────────────────────────
    # UI CONSTRUCTION
    # ─────────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # Title bar
        bar = tk.Frame(self.root, bg=BG_CARD, height=50)
        bar.pack(fill=tk.X)
        bar.pack_propagate(False)
        tk.Label(bar, text="  CyBot  ·  Invasive Fish Cluster Detector",
                 bg=BG_CARD, fg=FG_WHITE,
                 font=("Segoe UI", 13, "bold")).pack(side=tk.LEFT, padx=12, pady=8)
        self.lbl_conn = tk.Label(bar, text="● DISCONNECTED",
                                  bg=BG_CARD, fg=FG_RED,
                                  font=("Segoe UI", 10, "bold"))
        self.lbl_conn.pack(side=tk.RIGHT, padx=14)
        self.lbl_mode = tk.Label(bar, text="MODE: AUTONOMOUS",
                                  bg=BG_CARD, fg=FG_GREEN,
                                  font=("Segoe UI", 10, "bold"))
        self.lbl_mode.pack(side=tk.RIGHT, padx=10)

        # Body: sidebar | main
        body = tk.Frame(self.root, bg=BG_DARK)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=6)

        sidebar = tk.Frame(body, bg=BG_DARK, width=225)
        sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))
        sidebar.pack_propagate(False)

        main = tk.Frame(body, bg=BG_DARK)
        main.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self._build_sidebar(sidebar)
        self._build_main(main)

    # ── helpers ──────────────────────────────────────────────────────────────

    def _card(self, parent, title):
        return tk.LabelFrame(parent, text=f"  {title}  ",
                             bg=BG_PANEL, fg=FG_BLUE,
                             font=("Segoe UI", 8, "bold"),
                             bd=1, relief=tk.GROOVE)

    def _btn(self, parent, text, cmd, bg=BTN_BLUE, fg=FG_WHITE):
        return tk.Button(parent, text=text, command=cmd,
                         bg=bg, fg=fg, activebackground=bg,
                         relief=tk.FLAT, padx=6, pady=5,
                         font=("Segoe UI", 9, "bold"), cursor="hand2")

    # ── Sidebar ──────────────────────────────────────────────────────────────

    def _build_sidebar(self, parent):
        # Connection card
        cc = self._card(parent, "CONNECTION")
        cc.pack(fill=tk.X, pady=(0, 5))

        # Transport selector
        tk.Label(cc, text="Transport", bg=BG_PANEL, fg=FG_WHITE,
                 font=("Segoe UI", 8)).pack(anchor=tk.W, padx=8, pady=(6, 0))
        self.transport_var = tk.StringVar(value="TCP")
        trans_cb = ttk.Combobox(cc, textvariable=self.transport_var,
                                width=16, state="readonly",
                                values=["Serial", "TCP"])
        trans_cb.pack(padx=8, pady=2)
        trans_cb.bind("<<ComboboxSelected>>", self._on_transport_change)

        # ── Serial frame ───────────────────────────────────────
        self.serial_frame = tk.Frame(cc, bg=BG_PANEL)

        tk.Label(self.serial_frame, text="Port", bg=BG_PANEL, fg=FG_WHITE,
                 font=("Segoe UI", 8)).pack(anchor=tk.W, padx=8, pady=(6, 0))
        row = tk.Frame(self.serial_frame, bg=BG_PANEL)
        row.pack(fill=tk.X, padx=8)
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(row, textvariable=self.port_var,
                                      width=11, state="readonly")
        self.port_cb.pack(side=tk.LEFT)
        tk.Button(row, text="↻", command=self._refresh_ports,
                  bg=BG_CARD, fg=FG_BLUE, relief=tk.FLAT,
                  font=("Segoe UI", 11, "bold"),
                  cursor="hand2", padx=4).pack(side=tk.LEFT, padx=(3, 0))

        tk.Label(self.serial_frame, text="Baud Rate", bg=BG_PANEL, fg=FG_WHITE,
                 font=("Segoe UI", 8)).pack(anchor=tk.W, padx=8, pady=(4, 0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(self.serial_frame, textvariable=self.baud_var,
                     width=16, state="readonly",
                     values=["9600", "57600", "115200"]).pack(padx=8, pady=2)

        # ── TCP frame ──────────────────────────────────────────
        self.tcp_frame = tk.Frame(cc, bg=BG_PANEL)

        tk.Label(self.tcp_frame, text="CyBot IP", bg=BG_PANEL, fg=FG_WHITE,
                 font=("Segoe UI", 8)).pack(anchor=tk.W, padx=8, pady=(6, 0))
        self.ip_var = tk.StringVar(value="192.168.1.1")
        tk.Entry(self.tcp_frame, textvariable=self.ip_var,
                 bg=BG_CARD, fg=FG_WHITE, insertbackground=FG_WHITE,
                 relief=tk.FLAT, font=("Consolas", 9)).pack(
                    fill=tk.X, padx=8, pady=2, ipady=2)

        tk.Label(self.tcp_frame, text="TCP Port", bg=BG_PANEL, fg=FG_WHITE,
                 font=("Segoe UI", 8)).pack(anchor=tk.W, padx=8, pady=(4, 0))
        self.tcp_port_var = tk.StringVar(value="288")
        tk.Entry(self.tcp_frame, textvariable=self.tcp_port_var,
                 bg=BG_CARD, fg=FG_WHITE, insertbackground=FG_WHITE,
                 relief=tk.FLAT, font=("Consolas", 9)).pack(
                    fill=tk.X, padx=8, pady=2, ipady=2)

        # Show the default transport's frame
        self._on_transport_change()

        self.btn_conn = self._btn(cc, "Connect", self._toggle_conn, bg=BTN_GREEN)
        self.btn_conn.pack(fill=tk.X, padx=8, pady=(4, 8))

        # Mission card
        mc = self._card(parent, "MISSION")
        mc.pack(fill=tk.X, pady=(0, 5))
        buttons = [
            ("Start Roam (S)",       self._cmd_start,  BTN_GREEN),
            ("Manual Field Roam (R)", self._cmd_manual_roam, BTN_PURP),
            ("Manual Scan (M)",      self._cmd_scan,   BTN_BLUE),
            ("Go to Cluster (G)",    self._cmd_goto,   BTN_TEAL),
            ("Toggle Mode (T)",      self._cmd_toggle, BTN_PURP),
            ("ABORT / Stop (Q)",     self._cmd_abort,  BTN_RED),
        ]
        for text, cmd, color in buttons:
            self._btn(mc, text, cmd, bg=color).pack(fill=tk.X, padx=8, pady=2)
        tk.Frame(mc, bg=BG_PANEL, height=4).pack()

        # D-Pad card
        dc = self._card(parent, "MANUAL DRIVE")
        dc.pack(fill=tk.X, pady=(0, 5))
        tk.Label(dc, text="Keys: W (fwd)  A (left)  S (back)  D (right)",
                 bg=BG_PANEL, fg=FG_YELLOW,
                 font=("Segoe UI", 6), wraplength=200).pack(pady=(5, 2))

        pad = tk.Frame(dc, bg=BG_PANEL)
        pad.pack(pady=(0, 8))
        dpad_cfg = [
            ("▲", 0, 1, 'w'),
            ("◄", 1, 0, 'a'),
            ("►", 1, 2, 'd'),
            ("▼", 2, 1, 's'),
        ]
        for txt, row, col, key in dpad_cfg:
            tk.Button(pad, text=txt, bg=BG_CARD, fg=FG_WHITE, relief=tk.FLAT,
                      font=("Segoe UI", 12, "bold"), width=2, height=1,
                      cursor="hand2",
                      command=(lambda k=key: self._manual(k))
                      ).grid(row=row, column=col, padx=3, pady=3)

        self._btn(dc, "IMU Heading (H)", self._cmd_heading, bg=BTN_TEAL).pack(
            fill=tk.X, padx=8, pady=(0, 8))

        # Keyboard bindings (bot accepts 's' as backward in manual mode,
        # 's' as start-mission in autonomous mode — bot side handles routing)
        for k in ("w", "a", "d", "W", "A", "D"):
            self.root.bind(f"<{k}>", lambda _e, key=k: self._manual(key.lower()))
        for k in ("s", "S"):
            self.root.bind(f"<{k}>", lambda _e: self._cmd_start_or_back())
        for k in ("r", "R"):
            self.root.bind(f"<{k}>", lambda _e: self._cmd_manual_roam())
        for k in ("g", "G"):
            self.root.bind(f"<{k}>", lambda _e: self._cmd_goto())
        for k in ("t", "T"):
            self.root.bind(f"<{k}>", lambda _e: self._cmd_toggle())
        for k in ("h", "H"):
            self.root.bind(f"<{k}>", lambda _e: self._cmd_heading())
        for k in ("m", "M"):
            self.root.bind(f"<{k}>", lambda _e: self._cmd_scan())
        for k in ("q", "Q"):
            self.root.bind(f"<{k}>", lambda _e: self._cmd_abort())
        self.root.bind("<Escape>", lambda _e: self._cmd_abort())

        # Stats card
        sc = self._card(parent, "STATS")
        sc.pack(fill=tk.X, pady=(0, 5))
        self.stat_vars: dict = {}
        for label in ["Objects", "Clusters", "Target", "Dist Ahead", "Best Route",
                      "Heading", "Roll/Pitch", "IMU Calib", "IMU Status", "Status"]:
            row = tk.Frame(sc, bg=BG_PANEL)
            row.pack(fill=tk.X, padx=8, pady=1)
            tk.Label(row, text=label + ":", bg=BG_PANEL, fg=FG_WHITE,
                     font=("Segoe UI", 8), width=10, anchor=tk.W).pack(side=tk.LEFT)
            var = tk.StringVar(value="—")
            tk.Label(row, textvariable=var, bg=BG_PANEL, fg=FG_GREEN,
                     font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT)
            self.stat_vars[label] = var
        tk.Frame(sc, bg=BG_PANEL, height=4).pack()

    # ── Main panel ────────────────────────────────────────────────────────────

    def _build_main(self, parent):
        # Radar on top, log on bottom
        top = tk.Frame(parent, bg=BG_DARK)
        top.pack(fill=tk.BOTH, expand=True)

        bot = tk.Frame(parent, bg=BG_DARK, height=210)
        bot.pack(fill=tk.X, pady=(5, 0))
        bot.pack_propagate(False)

        # Radar
        radar_card = self._card(top, "SCAN VIEW — Top-Down (0°=right  90°=forward  180°=left)")
        radar_card.pack(fill=tk.BOTH, expand=True)

        fig = Figure(figsize=(8, 4.5), dpi=90, facecolor=BG_DARK)
        self.ax = fig.add_subplot(111)
        self.ax.set_facecolor("#060c18")
        self._draw_radar_grid()

        self.fig_canvas = FigureCanvasTkAgg(fig, master=radar_card)
        self.fig_canvas.draw()
        self.fig_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # Legend
        legend = tk.Frame(radar_card, bg=BG_PANEL)
        legend.pack(fill=tk.X, padx=4, pady=(0, 4))
        for color, label in [(FG_GREEN, "Fish / small object"),
                              (FG_RED,   "Large fish / diver"),
                              (FG_ORANGE,"Safe cluster target")]:
            tk.Label(legend, text=label, bg=BG_PANEL, fg=color,
                     font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=10)

        # Log
        log_card = self._card(bot, "SERIAL OUTPUT")
        log_card.pack(fill=tk.BOTH, expand=True)

        self.log = scrolledtext.ScrolledText(
            log_card, bg="#050a14", fg=FG_WHITE,
            font=("Courier New", 9), wrap=tk.WORD,
            state=tk.DISABLED, relief=tk.FLAT, bd=0
        )
        self.log.pack(fill=tk.BOTH, expand=True, padx=4, pady=(4, 0))
        self.log.tag_config("sys",    foreground=FG_BLUE)
        self.log.tag_config("ok",     foreground=FG_GREEN)
        self.log.tag_config("warn",   foreground=FG_YELLOW)
        self.log.tag_config("err",    foreground=FG_RED)
        self.log.tag_config("prompt", foreground=FG_ORANGE,
                             font=("Courier New", 10, "bold"))
        self.log.tag_config("data",   foreground="#445566")

        tk.Button(log_card, text="Clear Log", command=self._clear_log,
                  bg=BTN_GRAY, fg=FG_WHITE, relief=tk.FLAT,
                  font=("Segoe UI", 8), cursor="hand2",
                  padx=6, pady=2).pack(anchor=tk.E, padx=4, pady=(2, 4))

    # ─────────────────────────────────────────────────────────────────────────
    # RADAR VISUALIZATION
    # ─────────────────────────────────────────────────────────────────────────

    def _draw_radar_grid(self):
        ax = self.ax
        ax.set_xlim(-215, 215)
        ax.set_ylim(-30, 230)
        ax.set_aspect("equal")
        ax.axis("off")

        # Range rings
        for r in [50, 100, 150, 200]:
            angles = np.linspace(0, math.pi, 300)
            ax.plot(r * np.cos(angles), r * np.sin(angles),
                    color="#0d2030", lw=0.7, ls="--")
            ax.text(r + 3, 2, f"{r}cm", color="#1a3545", fontsize=5.5, va="bottom")

        # Angle spokes (0-180)
        for deg in range(0, 181, 30):
            rad = math.radians(deg)
            ax.plot([0, 208 * math.cos(rad)], [0, 208 * math.sin(rad)],
                    color="#0d2030", lw=0.5)
            lx = 218 * math.cos(rad)
            ly = 218 * math.sin(rad)
            ax.text(lx, ly, f"{deg}°", color="#253545",
                    ha="center", va="center", fontsize=5.5)

        # Robot body
        ax.add_patch(patches.FancyBboxPatch(
            (-10, -10), 20, 20, boxstyle="round,pad=2",
            facecolor=FG_BLUE, edgecolor="white", lw=0.6, zorder=10
        ))
        ax.text(0, 0, "BOT", color="white", ha="center", va="center",
                fontsize=6, fontweight="bold", zorder=11)

    def _compute_best_route(self):
        """Pick the clearest recent scan direction, biased toward straight ahead."""
        if len(self.scan_angles) < 2 or len(self.scan_ping) != len(self.scan_angles):
            return None

        safe_cm = 65.0
        min_candidate_cm = 35.0
        best = None

        for angle, dist in zip(self.scan_angles, self.scan_ping):
            if dist <= 0:
                continue

            clearance = min(float(dist), 250.0)
            offset = abs(float(angle) - 90.0)

            # Strongly prefer forward-ish routes when clearance is similar.
            score = clearance - (offset * 0.35)

            # Avoid choosing a direction that points through a detected object arc.
            for obj in self.objects:
                if obj["start_angle"] - 5 <= angle <= obj["end_angle"] + 5:
                    score -= 35.0
                    break

            if clearance < min_candidate_cm:
                score -= 80.0

            if best is None or score > best["score"]:
                best = {
                    "angle": float(angle),
                    "clearance": clearance,
                    "score": score,
                    "safe": clearance >= safe_cm,
                }

        return best

    def _route_text(self, route):
        if route is None:
            return "Scan needed"

        angle = route["angle"]
        clearance = route["clearance"]
        turn = angle - 90.0

        if clearance < 65.0:
            return f"Blocked, turn/scan ({clearance:.0f}cm)"
        if abs(turn) <= 7.5:
            return f"Forward clear ({clearance:.0f}cm)"
        if turn > 0:
            return f"Turn left {turn:.0f}° ({clearance:.0f}cm)"
        return f"Turn right {-turn:.0f}° ({clearance:.0f}cm)"

    def _update_radar(self):
        ax = self.ax
        ax.cla()
        ax.set_facecolor("#060c18")
        self._draw_radar_grid()
        self.best_route = self._compute_best_route()
        self.stat_vars["Best Route"].set(self._route_text(self.best_route))

        # ── Scan sweep (filled polygon) ──
        if len(self.scan_angles) >= 2 and len(self.scan_ping) == len(self.scan_angles):
            sx, sy = [0.0], [0.0]
            for ang, dist in zip(self.scan_angles, self.scan_ping):
                d = min(dist, 200.0)
                rad = math.radians(ang)
                sx.append(d * math.cos(rad))
                sy.append(d * math.sin(rad))
            sx.append(0.0); sy.append(0.0)
            ax.fill(sx, sy, color=FG_GREEN, alpha=0.05)
            ax.plot(sx[1:-1], sy[1:-1], color=FG_GREEN, lw=0.8, alpha=0.35)

        # ── Objects ──
        for obj in self.objects:
            mid_rad = math.radians(obj["mid_angle"])
            d       = min(obj["dist"], 198.0)
            x, y    = d * math.cos(mid_rad), d * math.sin(mid_rad)
            color   = FG_GREEN if obj["is_thin"] else FG_RED
            size    = 40       if obj["is_thin"] else 90
            label   = "Fish"   if obj["is_thin"] else "Diver"

            ax.scatter(x, y, c=color, s=size, zorder=6,
                       edgecolors="white", linewidths=0.5)
            ax.text(x, y + 10, label, color=color,
                    ha="center", fontsize=5.5, zorder=7)

            # Angular arc showing object extent
            s_rad = math.radians(obj["start_angle"])
            e_rad = math.radians(obj["end_angle"])
            arc   = np.linspace(s_rad, e_rad, 40)
            ax.plot(d * np.cos(arc), d * np.sin(arc),
                    color=color, lw=2.5, alpha=0.65, zorder=5)

        # ── Clusters ──
        for clust in self.clusters:
            mid_rad = math.radians(clust["mid_angle"])
            d       = min(clust["avg_dist"], 198.0)
            x, y    = d * math.cos(mid_rad), d * math.sin(mid_rad)

            ax.add_patch(patches.Circle(
                (x, y), radius=20, zorder=8,
                facecolor=FG_ORANGE, alpha=0.18,
                edgecolor=FG_ORANGE, lw=1.8
            ))
            ax.text(x, y + 25,
                    f"Cluster {clust['id']}\n{clust['count']} fish",
                    color=FG_ORANGE, ha="center",
                    fontsize=6, fontweight="bold", zorder=9)

        # Arrow to largest cluster
        if self.clusters:
            best = max(self.clusters, key=lambda c: c["count"])
            mid_rad = math.radians(best["mid_angle"])
            d       = min(best["avg_dist"], 198.0)
            ax.annotate(
                "", xy=(d * math.cos(mid_rad), d * math.sin(mid_rad)),
                xytext=(0, 0),
                arrowprops=dict(arrowstyle="->", color=FG_ORANGE, lw=1.2, alpha=0.5)
            )

        # ── Driver route recommendation ──
        if self.best_route:
            route = self.best_route
            angle = route["angle"]
            dist = min(route["clearance"], 190.0)
            rad = math.radians(angle)
            color = FG_GREEN if route["safe"] else FG_YELLOW
            x, y = dist * math.cos(rad), dist * math.sin(rad)

            ax.add_patch(patches.Wedge(
                (0, 0), dist, angle - 8, angle + 8,
                facecolor=color, alpha=0.12, edgecolor=color,
                linewidth=1.0, linestyle="--", zorder=4
            ))

            ax.annotate(
                "", xy=(x, y), xytext=(0, 0),
                arrowprops=dict(arrowstyle="simple", color=color,
                                lw=0, alpha=0.85, mutation_scale=22),
                zorder=12
            )
            ax.text(x, y + 14, "Best route", color=color,
                    ha="center", fontsize=8, fontweight="bold", zorder=13)

        self.fig_canvas.draw()

    # ─────────────────────────────────────────────────────────────────────────
    # CONNECTION (Serial or TCP)
    # ─────────────────────────────────────────────────────────────────────────

    def _on_transport_change(self, _event=None):
        """Show only the input fields relevant to the selected transport."""
        choice = self.transport_var.get()
        self.transport = choice
        if choice == "Serial":
            self.tcp_frame.pack_forget()
            self.serial_frame.pack(fill=tk.X)
        else:
            self.serial_frame.pack_forget()
            self.tcp_frame.pack(fill=tk.X)

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_var.set(ports[0])
        self._log(f"Ports available: {', '.join(ports) if ports else 'none'}", "sys")

    def _toggle_conn(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        if self.transport == "Serial":
            self._connect_serial()
        else:
            self._connect_tcp()

    def _connect_serial(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a serial port first.")
            return
        try:
            self.serial_conn = serial.Serial(port, int(self.baud_var.get()),
                                             timeout=0.05)
            self.connected = True
            self._on_connected(f"Serial  →  {port}  @  {self.baud_var.get()} baud")
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))

    def _connect_tcp(self):
        host = self.ip_var.get().strip()
        try:
            port = int(self.tcp_port_var.get())
        except ValueError:
            messagebox.showwarning("Bad port", "TCP port must be an integer.")
            return
        if not host:
            messagebox.showwarning("No host", "Enter the CyBot IP address.")
            return
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(4.0)         # connect timeout
            sock.connect((host, port))
            sock.settimeout(0.05)        # short read timeout for rx loop
            self.tcp_sock = sock
            self.connected = True
            self._on_connected(f"TCP     →  {host}:{port}")
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))

    def _on_connected(self, label: str):
        self.btn_conn.config(text="Disconnect", bg=BTN_RED)
        self.lbl_conn.config(text="● CONNECTED", fg=FG_GREEN)
        self._log(f"Connected  →  {label}", "ok")
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def _disconnect(self):
        self.connected = False
        try:
            if self.serial_conn:
                self.serial_conn.close()
        except Exception:
            pass
        try:
            if self.tcp_sock:
                self.tcp_sock.close()
        except Exception:
            pass
        self.serial_conn = None
        self.tcp_sock    = None
        self.btn_conn.config(text="Connect", bg=BTN_GREEN)
        self.lbl_conn.config(text="● DISCONNECTED", fg=FG_RED)
        self._log("Disconnected.", "warn")

    def _read_bytes(self) -> bytes:
        """Transport-agnostic non-blocking read. Returns b'' if no data."""
        if self.transport == "Serial" and self.serial_conn:
            return self.serial_conn.read(512)
        if self.transport == "TCP" and self.tcp_sock:
            try:
                return self.tcp_sock.recv(512)
            except socket.timeout:
                return b""
        return b""

    def _rx_loop(self):
        buf = ""
        while self.connected:
            try:
                raw = self._read_bytes()
                if raw:
                    buf += raw.decode("utf-8", errors="replace")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        self.rx_queue.put(line.rstrip("\r"))
                    # Detonation prompt has no trailing newline — flush immediately
                    if "Detonate? (y/n):" in buf:
                        self.rx_queue.put(buf.rstrip())
                        buf = ""
                    if "Ready>" in buf or "ManualRoam>" in buf:
                        self.rx_queue.put(buf.rstrip())
                        buf = ""
            except Exception:
                break

    # ─────────────────────────────────────────────────────────────────────────
    # PARSING
    # ─────────────────────────────────────────────────────────────────────────

    def _poll_queue(self):
        try:
            while True:
                self._handle_line(self.rx_queue.get_nowait())
        except queue.Empty:
            pass
        self.root.after(40, self._poll_queue)

    def _handle_line(self, line: str):
        s = line.strip()
        if not s:
            return

        # ── Detonation prompt ──────────────────────────────────────────────
        if "Detonate? (y/n):" in s:
            self._log(s, "prompt")
            self._detonation_dialog()
            return

        # ── Section headers ────────────────────────────────────────────────
        if "Scanning 0-180" in s:
            self._in_scan, self._in_objects, self._in_clusters = True, False, False
            self.scan_angles.clear(); self.scan_ping.clear(); self.scan_ir.clear()
            self.best_route = None
            self.stat_vars["Best Route"].set("Scanning...")
            self.stat_vars["Status"].set("Scanning field")
            self._log(s, "sys"); return

        if "DETECTED OBJECTS" in s:
            self._in_scan, self._in_objects, self._in_clusters = False, True, False
            self.objects.clear()
            self._log(s, "sys"); return

        if "THIN-PILLAR CLUSTERS" in s:
            self._in_scan, self._in_objects, self._in_clusters = False, False, True
            self.clusters.clear()
            self._log(s, "sys"); return

        if "Scan complete" in s:
            self._in_scan = False
            self._update_radar()
            self._log(s, "ok"); return

        # Skip separator lines
        if s.startswith("---") or s.startswith("===") or s.startswith("#"):
            self._log(s, "data"); return

        # ── Scan data row ──────────────────────────────────────────────────
        # " 90   |    12345       |  45.2  |  38.1"
        if self._in_scan:
            m = re.match(r"\s*(\d+)\s+\|\s+\d+\s+\|\s+([\d.]+)\s+\|\s+([\d.]+)", s)
            if m:
                self.scan_angles.append(float(m.group(1)))
                self.scan_ping.append(min(float(m.group(2)), 250.0))
                self.scan_ir.append(min(float(m.group(3)), 250.0))
                if len(self.scan_angles) >= 3 and len(self.scan_angles) % 10 == 0:
                    self._update_radar()
                self._log(s, "data"); return

        # ── Object row ─────────────────────────────────────────────────────
        # "  1 |  30  |  45  |  37  |   45.2   |  38.1  |   3.2  | THIN (fish)"
        if self._in_objects:
            m = re.match(
                r"\s*\d+\s+\|\s+(\d+)\s+\|\s+(\d+)\s+\|\s+(\d+)\s+\|\s+"
                r"([\d.]+)\s+\|\s+([\d.]+)\s+\|\s+([\d.]+)\s+\|\s+(.+)", s
            )
            if m:
                obj = {
                    "start_angle": int(m.group(1)),
                    "end_angle":   int(m.group(2)),
                    "mid_angle":   int(m.group(3)),
                    "dist":        float(m.group(4)),
                    "ir_dist":     float(m.group(5)),
                    "width":       float(m.group(6)),
                    "is_thin":     "THIN" in m.group(7),
                    "label":       m.group(7).strip(),
                }
                self.objects.append(obj)
                n_fish  = sum(1 for o in self.objects if     o["is_thin"])
                n_diver = sum(1 for o in self.objects if not o["is_thin"])
                self.stat_vars["Objects"].set(
                    f"{len(self.objects)}  ({n_fish} fish  {n_diver} diver)")
                self._update_radar()
                self._log(s, "data"); return

        # ── Cluster row ────────────────────────────────────────────────────
        # "  1 |    4   |       87       |    45.2"
        if self._in_clusters:
            m = re.match(r"\s*(\d+)\s+\|\s+(\d+)\s+\|\s+(\d+)\s+\|\s+([\d.]+)", s)
            if m:
                self.clusters.append({
                    "id":        int(m.group(1)),
                    "count":     int(m.group(2)),
                    "mid_angle": int(m.group(3)),
                    "avg_dist":  float(m.group(4)),
                })
                self.stat_vars["Clusters"].set(str(len(self.clusters)))
                self._update_radar()
                self._log(s, "data"); return

        # ── Target cluster summary ─────────────────────────────────────────
        m = re.search(r"TARGET: Cluster #(\d+).+?(\d+)\)", s)
        if m:
            self.stat_vars["Target"].set(f"#{m.group(1)}  ({m.group(2)} fish)")
            self._log(s, "ok"); return

        # ── Distance ahead ─────────────────────────────────────────────────
        m = re.search(r"Distance ahead: ([\d.]+) cm", s)
        if m:
            self.stat_vars["Dist Ahead"].set(f"{m.group(1)} cm")

        # ── IMU heading / calibration ─────────────────────────────────────
        m = re.search(
            r"IMU heading: ([\d.\-]+) deg \| roll ([\d.\-]+) \| pitch ([\d.\-]+) "
            r"\| calib S(\d+) G(\d+) A(\d+) M(\d+) \| status (\d+)", s
        )
        if m:
            self.stat_vars["Heading"].set(f"{m.group(1)}°")
            self.stat_vars["Roll/Pitch"].set(f"R {m.group(2)}° / P {m.group(3)}°")
            self.stat_vars["IMU Calib"].set(
                f"S{m.group(4)} G{m.group(5)} A{m.group(6)} M{m.group(7)}")
            self.stat_vars["IMU Status"].set(m.group(8))
            self._log(s, "data"); return

        if "IMU heading: unavailable" in s:
            self.stat_vars["Heading"].set("unavailable")
            self.stat_vars["IMU Calib"].set("—")
            self.stat_vars["IMU Status"].set("not detected")
            self._log(s, "warn"); return

        # ── Status / >>> lines ─────────────────────────────────────────────
        if s.startswith(">>>"):
            tag = "ok"
            if any(w in s for w in ("ERROR", "WARNING", "Lost")):
                tag = "err"
            elif any(w in s for w in ("hazard", "Boundary", "Hole", "Bump", "Diver nearby")):
                tag = "warn"

            if "Mode: AUTONOMOUS" in s:
                self.mode = "AUTO"
                self.lbl_mode.config(text="MODE: AUTONOMOUS", fg=FG_GREEN)
            elif "Mode: MANUAL" in s:
                self.mode = "MANUAL"
                self.lbl_mode.config(text="MODE: MANUAL", fg=FG_YELLOW)

            if "Mission COMPLETE" in s or "DETONATION CONFIRMED" in s:
                self.stat_vars["Status"].set("Done")
            if "Manual Field Roam" in s:
                self.stat_vars["Status"].set("Manual roam")
            if "TAPE WARNING" in s:
                self.stat_vars["Status"].set("Tape boundary")
            if "IMU WARNING" in s:
                self.stat_vars["Status"].set("IMU warning")
            if "ABORT" in s or "aborted" in s:
                self.stat_vars["Status"].set("Aborted")
                self.lbl_mode.config(text="MODE: READY", fg=FG_YELLOW)
            if ("Diver" in s and "nearby" in s) or "Large fish" in s:
                self.stat_vars["Status"].set("Large fish nearby")
            if "No safe clusters" in s:
                self.stat_vars["Status"].set("Searching")
            if "Playing detonation sound" in s:
                self.stat_vars["Status"].set("Detonation sound")
            if "cancelled" in s:
                self.stat_vars["Status"].set("Searching")

            self._log(s, tag); return

        # ── Everything else ────────────────────────────────────────────────
        self._log(s)

    # ─────────────────────────────────────────────────────────────────────────
    # DETONATION DIALOG
    # ─────────────────────────────────────────────────────────────────────────

    def _detonation_dialog(self):
        dlg = tk.Toplevel(self.root)
        dlg.title("Detonation Confirmation")
        dlg.geometry("460x260")
        dlg.configure(bg=BG_DARK)
        dlg.resizable(False, False)
        dlg.grab_set()
        dlg.focus_force()

        # Center over main window
        dlg.update_idletasks()
        ox = self.root.winfo_x() + (self.root.winfo_width()  - 460) // 2
        oy = self.root.winfo_y() + (self.root.winfo_height() - 260) // 2
        dlg.geometry(f"+{ox}+{oy}")

        tk.Label(dlg, text="CLUSTER DETECTED",
                 bg=BG_DARK, fg=FG_YELLOW,
                 font=("Segoe UI", 16, "bold")).pack(pady=(20, 6))
        tk.Label(dlg,
                 text="Small-fish cluster found.\nNo large fish detected in the blast radius.\nSound plays only after detonation is confirmed.",
                 bg=BG_DARK, fg=FG_WHITE,
                 font=("Segoe UI", 10)).pack()
        tk.Label(dlg, text="Confirm detonation?",
                 bg=BG_DARK, fg=FG_ORANGE,
                 font=("Segoe UI", 10, "bold")).pack(pady=(10, 0))

        bf = tk.Frame(dlg, bg=BG_DARK)
        bf.pack(pady=18)

        def yes():
            self._send("y")
            self._log(">>> YOU: Detonation CONFIRMED", "err")
            self.stat_vars["Status"].set("Detonated")
            dlg.destroy()

        def no():
            self._send("n")
            self._log(">>> YOU: Detonation cancelled — continuing search.", "warn")
            self.stat_vars["Status"].set("Searching")
            dlg.destroy()

        def abort():
            self._send("q")
            self._log(">>> YOU: Abort mission requested.", "err")
            self.stat_vars["Status"].set("Aborting")
            dlg.destroy()

        tk.Button(bf, text="DETONATE", command=yes,
                  bg=BTN_RED, fg=FG_WHITE, relief=tk.FLAT,
                  font=("Segoe UI", 12, "bold"),
                  padx=16, pady=9, cursor="hand2").pack(side=tk.LEFT, padx=8)
        tk.Button(bf, text="Skip", command=no,
                  bg=BTN_GRAY, fg=FG_WHITE, relief=tk.FLAT,
                  font=("Segoe UI", 12, "bold"),
                  padx=16, pady=9, cursor="hand2").pack(side=tk.LEFT, padx=8)
        tk.Button(bf, text="Abort", command=abort,
                  bg=BTN_RED, fg=FG_WHITE, relief=tk.FLAT,
                  font=("Segoe UI", 12, "bold"),
                  padx=16, pady=9, cursor="hand2").pack(side=tk.LEFT, padx=8)

        dlg.bind("<y>", lambda _e: yes())
        dlg.bind("<n>", lambda _e: no())
        dlg.bind("<q>", lambda _e: abort())
        dlg.bind("<Escape>", lambda _e: abort())

    # ─────────────────────────────────────────────────────────────────────────
    # COMMANDS
    # ─────────────────────────────────────────────────────────────────────────

    def _send(self, cmd: str):
        if not self.connected:
            self._log("Not connected — cannot send command.", "warn")
            return
        try:
            data = cmd.encode()
            if self.transport == "Serial" and self.serial_conn:
                self.serial_conn.write(data)
            elif self.transport == "TCP" and self.tcp_sock:
                self.tcp_sock.sendall(data)
            else:
                self._log("No transport open — cannot send command.", "warn")
        except Exception as exc:
            self._log(f"Send error: {exc}", "err")

    def _cmd_start(self):
        self.stat_vars["Status"].set("Roaming")
        self.mode = "AUTO"
        self.lbl_mode.config(text="MODE: AUTONOMOUS", fg=FG_GREEN)
        self._send("s")

    def _cmd_start_or_back(self):
        if self.mode == "MANUAL":
            self._manual("s")
        else:
            self._cmd_start()

    def _cmd_manual_roam(self):
        self.stat_vars["Status"].set("Manual roam")
        self.mode = "MANUAL"
        self.lbl_mode.config(text="MODE: MANUAL FIELD ROAM", fg=FG_YELLOW)
        self._send("r")

    def _cmd_scan(self):
        self.stat_vars["Status"].set("Scan requested")
        self._send("m")

    def _cmd_toggle(self):
        self._send("t")

    def _cmd_goto(self):
        self.stat_vars["Status"].set("Approaching")
        self._send("g")

    def _cmd_abort(self):
        self.stat_vars["Status"].set("Aborting")
        self._send("q")

    def _cmd_heading(self):
        self._send("h")

    def _manual(self, key: str):
        if self.mode != "MANUAL":
            self._log("Switch to MANUAL mode first (Toggle Mode button).", "warn")
            return
        self._send(key)

    # ─────────────────────────────────────────────────────────────────────────
    # LOG HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _log(self, text: str, tag: str = ""):
        self.log.configure(state=tk.NORMAL)
        self.log.insert(tk.END, text + "\n", (tag,) if tag else ())
        self.log.see(tk.END)
        self.log.configure(state=tk.DISABLED)

    def _clear_log(self):
        self.log.configure(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.configure(state=tk.DISABLED)


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()

    # Style ttk dropdowns to match dark theme
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TCombobox",
                    fieldbackground=BG_CARD, background=BG_CARD,
                    foreground=FG_WHITE, selectbackground=BG_CARD,
                    selectforeground=FG_WHITE, arrowcolor=FG_BLUE)

    app = CyBotGUI(root)
    app._refresh_ports()
    root.mainloop()


if __name__ == "__main__":
    main()

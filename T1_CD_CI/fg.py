# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Iterable, List, Tuple
from ipywidgets import interact
from matplotlib.widgets import Slider

# ---------------- DH CINEMATICA DIRECTA ---------------- #
def dh_to_T(alpha: float, a: float, d: float, theta: float, convention: str = "standard") -> np.ndarray:
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)

    conv = convention.strip().lower()
    if conv == "standard":
        T = np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)
    elif conv == "modified":
        T = np.array([
            [ct, -st, 0.0, a],
            [st*ca, ct*ca, -sa, -sa*d],
            [st*sa, ct*sa, ca, ca*d],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)
    else:
        raise ValueError("Convención desconocida")
    return T

def dh_kinematics(DH: np.ndarray, jointTypes: Iterable, q: Iterable, 
                  convention: str = "standard", do_plot: bool = True, 
                  frame_scale: float = None, joint_signs=None) -> Tuple[List[np.ndarray], List[np.ndarray], np.ndarray]:

    DH = np.asarray(DH, dtype=float)
    if DH.ndim != 2 or DH.shape[1] != 4:
        raise ValueError("DH debe ser Nx4 [alpha, a, d, theta]")
    N = DH.shape[0]

    if isinstance(jointTypes, str):
        jt = jointTypes.upper()
    else:
        jt = "".join(str(s)[0] for s in jointTypes).upper()
    if len(jt) != N:
        raise ValueError("jointTypes debe tener longitud N")

    q = np.asarray(q, dtype=float).reshape(-1)
    if q.size != N:
        raise ValueError("q debe tener longitud N")

    if joint_signs is None:
        joint_signs = np.ones(N)

    alpha = DH[:,0].copy()
    a = DH[:,1].copy()
    d = DH[:,2].copy()
    theta = DH[:,3].copy()

    for i in range(N):
        if jt[i] == "R":
            theta[i] = theta[i] + joint_signs[i]*q[i]
        elif jt[i] == "P":
            d[i] = d[i] + joint_signs[i]*q[i]
        else:
            raise ValueError("jointTypes solo admite 'R' o 'P'")

    # Matrices A_i
    A_i = [dh_to_T(alpha[i], a[i], d[i], theta[i], convention) for i in range(N)]

    # Acumuladas T0->i
    T_0_i = [np.eye(4)]
    for i in range(N):
        T_0_i.append(T_0_i[-1] @ A_i[i])
    T_0_N = T_0_i[-1]

    if do_plot:
        if frame_scale is None:
            L = float(np.sum(np.abs(a)) + np.sum(np.abs(d)))
            frame_scale = max(1e-3, 0.15*L)
        plot_frames_3d(T_0_i, frame_scale)

    return A_i, T_0_i, T_0_N

# ----------------- PLOTEO ----------------- #
def plot_frames_3d(T_0_i: List[np.ndarray], s: float = 0.1) -> None:
    N = len(T_0_i)-1
    origins = np.zeros((N+1,3))
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection="3d")

    for i, T in enumerate(T_0_i):
        o = T[0:3,3]
        R = T[0:3,0:3]
        origins[i,:] = o
        ax.quiver(o[0], o[1], o[2], s*R[0,0], s*R[1,0], s, color='r', linewidth=1.5)
        ax.quiver(o[0], o[1], o[2], s*R[0,1], s*R[1,1], s*R[2,1], color='g', linewidth=1.5)
        ax.quiver(o[0], o[1], o[2], s*R[0,2], s*R[1,2], s*R[2,2], color='b', linewidth=1.5)
        ax.text(o[0], o[1], o[2], f'{{{i}}}', fontsize=10)

    ax.plot(origins[:,0], origins[:,1], origins[:,2], '-k', linewidth=1.2)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title('Frames DH')
    ax.view_init(elev=28, azim=135)
    plt.tight_layout()
    plt.show()

# ----------------- EJEMPLO CON SLIDER ----------------- #
DH = np.array([
    [-np.pi/2, 0.0, 0.065, 0.0],
    [0.0, 0.095, 0.0, 0.0],
    [np.pi/2, 0.055, 0.0, 0.0],
])
types = ["R","R","R"]
joint_signs = [1,-1,1]

# Ventana inicial
fig = plt.figure(figsize=(6,6))
ax_slider = plt.axes([0.2, 0.01, 0.6, 0.03])
slider = Slider(ax_slider, 'q1°', 0, 180, valinit=90)
slider2 = Slider(ax_slider, 'q2°', 0, 180, valinit=90)
slider3 = Slider(ax_slider, 'q3°', 0, 180, valinit=90)

def update(val):
    q1_deg = slider.val
    q = [np.radians(q1_deg), np.radians(q1_deg), np.radians(q1_deg)]
    plt.clf()  # limpia la figura
    dh_kinematics(DH, types, q, joint_signs=joint_signs, do_plot=True)

slider.on_changed(update)
plt.show()

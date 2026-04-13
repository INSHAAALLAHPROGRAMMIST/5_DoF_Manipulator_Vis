# ===============================================================================
# ИМПОРТ БИБЛИОТЕК / KUTUBXONALARNI IMPORT QILISH
# ===============================================================================
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button
from fractions import Fraction
import textwrap
import math

# ===============================================================================
# ФУНКЦИИ ДЛЯ ТОЧНЫХ ВЫЧИСЛЕНИЙ
# ===============================================================================
def to_fraction(value, max_denominator=1000000):
    """Float yoki stringni aniq kasrga aylantiradi"""
    if isinstance(value, str):
        if '/' in value:
            parts = value.split('/')
            return Fraction(int(parts[0]), int(parts[1]))
        else:
            return Fraction(float(value)).limit_denominator(max_denominator)
    else:
        return Fraction(value).limit_denominator(max_denominator)

def format_value(value):
    """Sonni kasr ko'rinishida formatlaydi"""
    frac = to_fraction(float(value), max_denominator=1000000)
    if frac.denominator == 1:
        return str(frac.numerator)
    else:
        return f"{frac.numerator}/{frac.denominator}"

def parse_input(value_str):
    """Kasr yoki oddiy sonni floatga aylantiradi"""
    value_str = value_str.strip()
    if '/' in value_str:
        parts = value_str.split('/')
        return float(Fraction(int(parts[0]), int(parts[1])))
    else:
        return float(Fraction(value_str))

# ===============================================================================
# ROBOT O'LCHAMLARI VA CHEKLOVLARI
# ===============================================================================
L = Fraction(30, 1)

# h_CB cheklovlari
H_CB_MIN = Fraction(30, 1)
H_CB_MAX = Fraction(57, 1)

# Burchak cheklovlari
BETA_MIN_DEG = Fraction(27, 1)
BETA_MAX_DEG = Fraction(120, 1)
GAMMA_MIN_DEG = Fraction(-60, 1)
GAMMA_MAX_DEG = Fraction(120, 1)
ELBOW_MIN_DEG = Fraction(30, 1)
ELBOW_MAX_DEG = Fraction(160, 1)

# ===============================================================================
# YORDAMCHI FUNKSIYA: Tirsak burchagini hisoblash
# ===============================================================================
def calc_elbow_angle_from_angles(beta_deg, gamma_deg):
    """β va γ burchaklaridan tirsak burchagini hisoblash"""
    return 180.0 - abs(float(beta_deg) - float(gamma_deg))

# ===============================================================================
# TESKARI KINEMATIKA (IK) - KASRLAR BILAN
# ===============================================================================
def inverse_kinematics(E_x, E_y, E_z, n_x, n_y, n_z):
    """
    TESKARI KINEMATIKA - KASRLAR BILAN ANIQ HISOBLASH
    """
    
    # Floatga aylantirish
    E_x = float(E_x)
    E_y = float(E_y)
    E_z = float(E_z)
    n_x = float(n_x)
    n_y = float(n_y)
    n_z = float(n_z)
    
    # 1. n vektorini normalizatsiya qilish
    vec_len = np.sqrt(n_x**2 + n_y**2 + n_z**2)
    if vec_len < 1e-10:
        return None, None, None, None, None, None, None, None, None, None, False, "n vektori nolga teng"
    
    n_x, n_y, n_z = n_x/vec_len, n_y/vec_len, n_z/vec_len
    
    # 2. E nuqtasi
    E = np.array([E_x, E_y, E_z])
    
    # 3. D nuqtasini hisoblash
    L_float = float(L)
    D = E - L_float * np.array([n_x, n_y, n_z])
    
    # 4. Alpha burchagini hisoblash
    alpha = np.arctan2(D[1], D[0])
    
    # 5. Gamma burchagini n_z dan hisoblash
    gamma = np.arcsin(np.clip(n_z, -1.0, 1.0))
    gamma_deg = np.degrees(gamma)
    
    gamma_alt = np.pi - gamma if gamma >= 0 else -np.pi - gamma
    gamma_alt_deg = np.degrees(gamma_alt)
    
    # 6. Beta ni topish
    if abs(np.cos(alpha)) > 1e-6:
        cd_h = (D[0] - 0) / np.cos(alpha)
    else:
        cd_h = D[1] / np.sin(alpha) if abs(np.sin(alpha)) > 1e-6 else 0
    
    D_h = abs(cd_h)
    
    if D_h > L_float + 1e-6:
        return None, None, None, None, None, None, None, None, None, None, False, \
               f"D gorizontal masofa ({format_value(D_h)} sm) > L ({format_value(L_float)} sm). Yetib borolmaydi."
    
    beta1 = np.arccos(np.clip(D_h / L_float, -1.0, 1.0))
    beta2 = 2*np.pi - beta1
    
    beta1_deg = np.degrees(beta1)
    beta2_deg = np.degrees(beta2)
    
    best_result = None
    best_score = float('inf')
    
    for beta_test, beta_deg in [(beta1, beta1_deg), (beta2, beta2_deg)]:
        if beta_deg < float(BETA_MIN_DEG) or beta_deg > float(BETA_MAX_DEG):
            continue
        
        cd_v = L_float * np.sin(beta_test)
        h_CB_test = D[2] - cd_v
        
        if h_CB_test < float(H_CB_MIN) or h_CB_test > float(H_CB_MAX):
            continue
        
        oa_test = D[0] - L_float * np.cos(beta_test) * np.cos(alpha)
        
        for gamma_test, gamma_test_deg in [(gamma, gamma_deg), (gamma_alt, gamma_alt_deg)]:
            if gamma_test_deg < float(GAMMA_MIN_DEG) or gamma_test_deg > float(GAMMA_MAX_DEG):
                continue
            
            n_xy = np.cos(gamma_test)
            n_x_exp = n_xy * np.cos(alpha)
            n_y_exp = n_xy * np.sin(alpha)
            
            if n_x * n_x_exp + n_y * n_y_exp < 0.9:
                continue
            
            elbow_deg = calc_elbow_angle_from_angles(beta_deg, gamma_test_deg)
            if elbow_deg < float(ELBOW_MIN_DEG) or elbow_deg > float(ELBOW_MAX_DEG):
                continue
            
            C_test = np.array([oa_test, 0.0, h_CB_test])
            D_test = C_test + np.array([L_float * np.cos(beta_test) * np.cos(alpha),
                                         L_float * np.cos(beta_test) * np.sin(alpha),
                                         L_float * np.sin(beta_test)])
            
            if np.linalg.norm(D_test - D) > 0.5:
                continue
            
            score = abs(h_CB_test - 45.0)
            if score < best_score:
                best_result = (oa_test, h_CB_test, beta_test, gamma_test, C_test, beta_deg, gamma_test_deg, elbow_deg)
                best_score = score
    
    if best_result is None:
        return None, None, None, None, None, None, None, None, None, None, False, \
               "Yetib bo'lmaydigan nuqta! Cheklovlardan tashqarida."
    
    oa, h_CB, beta, gamma, C, beta_deg, gamma_deg, elbow_deg = best_result
    
    A = np.array([oa, 0.0, 0.0])
    B = A.copy()
    
    return A, B, C, D, E, alpha, beta, gamma, h_CB, oa, True, "OK"


# ===============================================================================
# 3D OBYEKTLARNI YARATISH
# ===============================================================================
def create_platform_vertices(cx, w=40, ln=25, h=15):
    hw, hl = w/2, ln/2
    return np.array([
        [cx-hw, -hl, 0], [cx+hw, -hl, 0], [cx+hw, hl, 0], [cx-hw, hl, 0],
        [cx-hw, -hl, h], [cx+hw, -hl, h], [cx+hw, hl, h], [cx-hw, hl, h]
    ])

def create_fence_vertices(x_off=70, y_off=35, w=120, ln=40, h=60):
    hw, hl = w/2, ln/2
    return np.array([
        [x_off-hw, y_off-hl, 0], [x_off+hw, y_off-hl, 0],
        [x_off+hw, y_off+hl, 0], [x_off-hw, y_off+hl, 0],
        [x_off-hw, y_off-hl, h], [x_off+hw, y_off-hl, h],
        [x_off+hw, y_off+hl, h], [x_off-hw, y_off+hl, h]
    ])


# ===============================================================================
# BOSHLANG'ICH PARAMETRLAR (Kasr formatida)
# ===============================================================================
E_x0 = Fraction(30, 1)
E_y0 = Fraction(0, 1)
E_z0 = Fraction(75, 1)
n_x0 = Fraction(1, 1)
n_y0 = Fraction(0, 1)
n_z0 = Fraction(0, 1)

current_E_x = float(E_x0)
current_E_y = float(E_y0)
current_E_z = float(E_z0)
current_n_x = float(n_x0)
current_n_y = float(n_y0)
current_n_z = float(n_z0)

# ===============================================================================
# GRAFIK OYNA YARATISH
# ===============================================================================
plt.close('all')
fig = plt.figure(figsize=(16, 10))
ax = fig.add_axes([0.26, 0.05, 0.72, 0.90], projection='3d')
ax.set_facecolor('#f5f5f5')

# Koordinata o'qlari
ax.plot([-100, 200], [0, 0], [0, 0], color='red', linewidth=1, alpha=0.5)
ax.plot([0, 0], [-100, 100], [0, 0], color='green', linewidth=1, alpha=0.5)
ax.plot([0, 0], [0, 0], [-10, 140], color='blue', linewidth=1, alpha=0.5)

# Boshlang'ich holatni hisoblash
A, B, C, D, E, alpha, beta, gamma, h_CB, oa, valid, msg = inverse_kinematics(
    current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
)

if not valid:
    print(f"XATO: {msg}")
    A = np.array([0, 0, 0])
    B = A.copy()
    C = np.array([0, 0, 45])
    D = np.array([0, 0, 75])
    E = np.array([30, 0, 75])
    alpha, beta, gamma, h_CB, oa = 0, np.radians(90), 0, 45, 0

beta_deg = np.degrees(beta)
gamma_deg = np.degrees(gamma)
elbow_deg = calc_elbow_angle_from_angles(beta_deg, gamma_deg)

# ===============================================================================
# PLATFORMA
# ===============================================================================
pv = create_platform_vertices(oa)
platform_lines = []

for face in [[0,1,2,3,0], [4,5,6,7,4]]:
    line, = ax.plot(pv[face,0], pv[face,1], pv[face,2], color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

for i in range(4):
    line, = ax.plot([pv[i,0], pv[i+4,0]], [pv[i,1], pv[i+4,1]], [pv[i,2], pv[i+4,2]], 
                    color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

# ===============================================================================
# POMIDOR EGATLARI
# ===============================================================================
for y_off in [-50, 50]:
    fv = create_fence_vertices(x_off=70, y_off=y_off)
    for face in [[0,1,2,3,0], [4,5,6,7,4]]:
        ax.plot(fv[face,0], fv[face,1], fv[face,2], color='brown', linewidth=2, alpha=0.7)
    for i in range(4):
        ax.plot([fv[i,0], fv[i+4,0]], [fv[i,1], fv[i+4,1]], [fv[i,2], fv[i+4,2]], 
                color='brown', linewidth=2, alpha=0.7)

# ===============================================================================
# MANIPULYATOR BO'G'INLARI
# ===============================================================================
bc_line, = ax.plot([A[0], C[0]], [A[1], C[1]], [A[2], C[2]], color='black', linewidth=3, label='BC')
cd_line, = ax.plot([C[0], D[0]], [C[1], D[1]], [C[2], D[2]], color='orange', linewidth=3, marker='o', label='CD')
de_line, = ax.plot([D[0], E[0]], [D[1], E[1]], [D[2], E[2]], color='blue', linewidth=3, marker='o', label='DE')
end_sc = ax.scatter(E[0], E[1], E[2], color='purple', s=100, marker='*', label='E')

# ===============================================================================
# SAHNA SOZLAMALARI
# ===============================================================================
ax.set_xlim(-80, 180)
ax.set_ylim(-130, 130)
ax.set_zlim(-10, 140)
ax.set_box_aspect([260, 260, 150])
ax.set_xlabel('X (sm)', fontsize=11)
ax.set_ylabel('Y (sm)', fontsize=11)
ax.set_zlabel('Z (sm)', fontsize=11)
ax.view_init(elev=20, azim=-60)
ax.legend(loc='upper left', fontsize=9)
ax.set_title(f"IK | β:[{format_value(BETA_MIN_DEG)}°,{format_value(BETA_MAX_DEG)}°] γ:[{format_value(GAMMA_MIN_DEG)}°,{format_value(GAMMA_MAX_DEG)}°] Tirsak:[{format_value(ELBOW_MIN_DEG)}°,{format_value(ELBOW_MAX_DEG)}°]")

# ===============================================================================
# MA'LUMOT PANELLARI (Kasr formatida)
# ===============================================================================
info_ax = fig.add_axes([0.02, 0.42, 0.20, 0.20])
info_ax.text(0.05, 0.5, 
    f"E_x = {format_value(current_E_x)}\nE_y = {format_value(current_E_y)}\nE_z = {format_value(current_E_z)}\n"
    f"n_x = {format_value(current_n_x)}\nn_y = {format_value(current_n_y)}\nn_z = {format_value(current_n_z)}",
    fontsize=10, va='center', fontweight='bold')
info_ax.set_xlim(0,1)
info_ax.set_ylim(0,1)
info_ax.axis('off')

coord_ax = fig.add_axes([0.02, 0.05, 0.20, 0.32])
coord_ax.text(0.05, 0.5, 
    f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\n"
    f"B: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\n"
    f"C: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\n"
    f"D: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\n"
    f"E: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\n"
    f"α: {format_value(np.degrees(alpha))}°\nβ: {format_value(beta_deg)}°\nγ: {format_value(gamma_deg)}°\n"
    f"h_CB: {format_value(h_CB)} sm\nOA: {format_value(oa)} sm\nTirsak: {format_value(elbow_deg)}°",
    fontsize=9, va='center', fontweight='bold')
coord_ax.set_xlim(0,1)
coord_ax.set_ylim(0,1)
coord_ax.axis('off')

# ===============================================================================
# BOSHQARUV ELEMENTLARI (Kasr kiritish uchun)
# ===============================================================================
ex_ax = plt.axes([0.08, 0.87, 0.08, 0.025])
ey_ax = plt.axes([0.08, 0.83, 0.08, 0.025])
ez_ax = plt.axes([0.08, 0.79, 0.08, 0.025])
nx_ax = plt.axes([0.08, 0.75, 0.08, 0.025])
ny_ax = plt.axes([0.08, 0.71, 0.08, 0.025])
nz_ax = plt.axes([0.08, 0.67, 0.08, 0.025])

button_ax = plt.axes([0.09, 0.61, 0.10, 0.035])
btn = Button(button_ax, '▶ YURISH', color='lightgreen', hovercolor='lime')

# Yorliqlar
fig.text(0.02, 0.88, 'E_x:', fontsize=10, fontweight='bold', va='center')
fig.text(0.02, 0.84, 'E_y:', fontsize=10, fontweight='bold', va='center')
fig.text(0.02, 0.80, 'E_z:', fontsize=10, fontweight='bold', va='center')
fig.text(0.02, 0.76, 'n_x:', fontsize=10, fontweight='bold', va='center')
fig.text(0.02, 0.72, 'n_y:', fontsize=10, fontweight='bold', va='center')
fig.text(0.02, 0.68, 'n_z:', fontsize=10, fontweight='bold', va='center')

# Hintlar
fig.text(0.17, 0.88, "(cheksiz)", fontsize=7, color='gray', va='center')
fig.text(0.17, 0.84, "(cheksiz)", fontsize=7, color='gray', va='center')
fig.text(0.17, 0.80, "(cheksiz)", fontsize=7, color='gray', va='center')
fig.text(0.17, 0.76, "[-1,1]", fontsize=7, color='gray', va='center')
fig.text(0.17, 0.72, "[-1,1]", fontsize=7, color='gray', va='center')
fig.text(0.17, 0.68, "[-1,1]", fontsize=7, color='gray', va='center')

ex_tb = TextBox(ex_ax, '', initial=format_value(E_x0))
ey_tb = TextBox(ey_ax, '', initial=format_value(E_y0))
ez_tb = TextBox(ez_ax, '', initial=format_value(E_z0))
nx_tb = TextBox(nx_ax, '', initial=format_value(n_x0))
ny_tb = TextBox(ny_ax, '', initial=format_value(n_y0))
nz_tb = TextBox(nz_ax, '', initial=format_value(n_z0))

# ===============================================================================
# XATOLIK PANELI
# ===============================================================================
err_ax = fig.add_axes([0.02, 0.92, 0.20, 0.06])
err_ax.set_facecolor('#fafafa')
for spine in err_ax.spines.values():
    spine.set_visible(True)
    spine.set_color('#cccccc')
    spine.set_linewidth(1)

err_ax.text(0.5, 0.5, "✓ Holat: Tayyor", fontsize=9, va='center', ha='center', 
            fontweight='bold', color='green', transform=err_ax.transAxes, wrap=True)
err_ax.set_xlim(0, 1)
err_ax.set_ylim(0, 1)
err_ax.set_xticks([])
err_ax.set_yticks([])


def update_error(msg, col):
    err_ax.clear()
    for spine in err_ax.spines.values():
        spine.set_visible(True)
        spine.set_color('#cccccc')
        spine.set_linewidth(1)
    
    wrapped_msg = textwrap.fill(msg, width=30)
    
    err_ax.text(0.5, 0.5, wrapped_msg, fontsize=8, va='center', ha='center', 
                fontweight='bold', color=col, transform=err_ax.transAxes, wrap=True)
    err_ax.set_xlim(0, 1)
    err_ax.set_ylim(0, 1)
    err_ax.set_xticks([])
    err_ax.set_yticks([])
    fig.canvas.draw()


# ===============================================================================
# ANIMATSIYA FUNKSIYALARI
# ===============================================================================
def start_anim(event):
    try:
        tEx = parse_input(ex_tb.text)
        tEy = parse_input(ey_tb.text)
        tEz = parse_input(ez_tb.text)
        tnx = parse_input(nx_tb.text)
        tny = parse_input(ny_tb.text)
        tnz = parse_input(nz_tb.text)
        animate(tEx, tEy, tEz, tnx, tny, tnz)
    except (ValueError, ZeroDivisionError) as e:
        update_error(f"✗ XATO: Noto'g'ri son formati! ({str(e)})", 'red')

def animate(tEx, tEy, tEz, tnx, tny, tnz):
    global current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
    
    At, Bt, Ct, Dt, Et, al, bt, gm, hCB, oa, vld, msg = inverse_kinematics(tEx, tEy, tEz, tnx, tny, tnz)
    
    if not vld:
        update_error(f"✗ {msg}", 'red')
        return
    
    update_error("⟳ Animatsiya bajarilmoqda...", 'blue')
    
    steps = 30
    dEx = (tEx - current_E_x) / steps
    dEy = (tEy - current_E_y) / steps
    dEz = (tEz - current_E_z) / steps
    dnx = (tnx - current_n_x) / steps
    dny = (tny - current_n_y) / steps
    dnz = (tnz - current_n_z) / steps
    
    for _ in range(steps):
        current_E_x += dEx
        current_E_y += dEy
        current_E_z += dEz
        current_n_x += dnx
        current_n_y += dny
        current_n_z += dnz
        
        A, B, C, D, E, alpha, beta, gamma, h_CB, oa, vld, _ = inverse_kinematics(
            current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
        )
        if not vld:
            continue
        
        beta_deg = np.degrees(beta)
        gamma_deg = np.degrees(gamma)
        elbow = calc_elbow_angle_from_angles(beta_deg, gamma_deg)
        
        pv = create_platform_vertices(oa)
        platform_lines[0].set_data(pv[[0,1,2,3,0],0], pv[[0,1,2,3,0],1])
        platform_lines[0].set_3d_properties(pv[[0,1,2,3,0],2])
        platform_lines[1].set_data(pv[[4,5,6,7,4],0], pv[[4,5,6,7,4],1])
        platform_lines[1].set_3d_properties(pv[[4,5,6,7,4],2])
        for j in range(4):
            platform_lines[j+2].set_data([pv[j,0], pv[j+4,0]], [pv[j,1], pv[j+4,1]])
            platform_lines[j+2].set_3d_properties([pv[j,2], pv[j+4,2]])
        
        bc_line.set_data([A[0], C[0]], [A[1], C[1]])
        bc_line.set_3d_properties([A[2], C[2]])
        cd_line.set_data([C[0], D[0]], [C[1], D[1]])
        cd_line.set_3d_properties([C[2], D[2]])
        de_line.set_data([D[0], E[0]], [D[1], E[1]])
        de_line.set_3d_properties([D[2], E[2]])
        end_sc._offsets3d = ([E[0]], [E[1]], [E[2]])
        
        info_ax.clear()
        info_ax.text(0.05, 0.5, 
            f"E_x = {format_value(current_E_x)}\nE_y = {format_value(current_E_y)}\nE_z = {format_value(current_E_z)}\n"
            f"n_x = {format_value(current_n_x)}\nn_y = {format_value(current_n_y)}\nn_z = {format_value(current_n_z)}",
            fontsize=10, va='center', fontweight='bold')
        info_ax.set_xlim(0,1)
        info_ax.set_ylim(0,1)
        info_ax.axis('off')
        
        coord_ax.clear()
        coord_ax.text(0.05, 0.5, 
            f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\n"
            f"B: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\n"
            f"C: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\n"
            f"D: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\n"
            f"E: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\n"
            f"α: {format_value(np.degrees(alpha))}°\nβ: {format_value(beta_deg)}°\nγ: {format_value(gamma_deg)}°\n"
            f"h_CB: {format_value(h_CB)} sm\nOA: {format_value(oa)} sm\nTirsak: {format_value(elbow)}°",
            fontsize=9, va='center', fontweight='bold')
        coord_ax.set_xlim(0,1)
        coord_ax.set_ylim(0,1)
        coord_ax.axis('off')
        
        fig.canvas.draw()
        fig.canvas.flush_events()
    
    current_E_x, current_E_y, current_E_z = tEx, tEy, tEz
    current_n_x, current_n_y, current_n_z = tnx, tny, tnz
    update_error("✓ Holat: Tayyor!", 'green')

btn.on_clicked(start_anim)
plt.show()
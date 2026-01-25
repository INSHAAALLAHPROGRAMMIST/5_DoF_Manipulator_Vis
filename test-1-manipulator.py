import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button
import time

L = 30.0
h_DC = 30.0

def fk_cpp(alpha_deg, beta_deg, gamma_deg, h_CB, oa_dist):
    alpha = np.radians(alpha_deg)
    beta  = np.radians(beta_deg)
    gamma = np.radians(gamma_deg)

    Ex = oa_dist + L * (np.cos(alpha) * np.cos(beta) - np.cos(alpha) * np.cos(gamma + beta))
    Ey = L * (np.sin(alpha) * np.cos(beta) - np.sin(alpha) * np.cos(gamma + beta))
    Ez = L * (np.sin(beta) - np.sin(gamma + beta)) + h_CB
    return np.array([Ex, Ey, Ez])

def compute_chain(alpha_deg, beta_deg, gamma_deg, h_CB, oa_dist):
    alpha = np.radians(alpha_deg)
    beta  = np.radians(beta_deg)

    # Manipulyator platformaning ustida, y o'qiga yaqinroq joyda turadi
    # platform_length = 25, shuning uchun yarim uzunlik = 12.5
    # Yarim uzunlikning yarmi = 6.25, shu masofada y o'qiga yaqinroq
    platform_height = 15
    manipulator_y_offset = 6.25  # y o'qiga yaqinroq
    
    A = np.array([oa_dist, manipulator_y_offset, platform_height])
    B = A.copy()
    C = np.array([oa_dist, manipulator_y_offset, platform_height + h_CB])

    Dx = oa_dist + np.cos(beta) * h_DC * np.cos(alpha)
    Dy = manipulator_y_offset + np.cos(beta) * h_DC * np.sin(alpha)
    Dz = platform_height + h_CB + np.sin(beta) * h_DC
    D = np.array([Dx, Dy, Dz])

    # E nuqtasini ham yangi A nuqtasiga nisbatan hisoblash
    Ex = oa_dist + L * (np.cos(alpha) * np.cos(beta) - np.cos(alpha) * np.cos(np.radians(gamma_deg) + beta))
    Ey = manipulator_y_offset + L * (np.sin(alpha) * np.cos(beta) - np.sin(alpha) * np.cos(np.radians(gamma_deg) + beta))
    Ez = platform_height + L * (np.sin(beta) - np.sin(np.radians(gamma_deg) + beta)) + h_CB
    E = np.array([Ex, Ey, Ez])
    
    return A, B, C, D, E

def create_platform_vertices(center_x, platform_width=40, platform_length=25, platform_height=15):
    """Parallelepiped platformaning uchlarini yaratish"""
    hw = platform_width / 2   # yarim kenglik (x o'qi bo'yicha)
    hl = platform_length / 2  # yarim uzunlik (y o'qi bo'yicha) - qisqaroq
    
    # Platformaning pastki qismi (z = 0 dan boshlanadi)
    vertices = np.array([
        [center_x - hw, -hl, 0],                    # 0: chap-orqa-past
        [center_x + hw, -hl, 0],                    # 1: o'ng-orqa-past
        [center_x + hw,  hl, 0],                    # 2: o'ng-old-past
        [center_x - hw,  hl, 0],                    # 3: chap-old-past
        [center_x - hw, -hl, platform_height],     # 4: chap-orqa-yuqori
        [center_x + hw, -hl, platform_height],     # 5: o'ng-orqa-yuqori
        [center_x + hw,  hl, platform_height],     # 6: o'ng-old-yuqori
        [center_x - hw,  hl, platform_height]      # 7: chap-old-yuqori
    ])
    return vertices

def create_fence_vertices(fence_width=120, fence_length=40, fence_height=60, x_offset=70, y_offset=35):
    """Ogorod (to'siq) parallelepiped uchlarini yaratish - bir joyda turadi"""
    hw = fence_width / 2   # yarim kenglik (x o'qi bo'yicha - uzunroq)
    hl = fence_length / 2  # yarim uzunlik (y o'qi bo'yicha)
    
    # Ogorodning pastki qismi (z = 0 dan boshlanadi)
    # x_offset - X o'qining musbat tomonida, manipulyatordan uzunroq intervalda
    vertices = np.array([
        [x_offset - hw, y_offset - hl, 0],                    # 0: chap-orqa-past
        [x_offset + hw, y_offset - hl, 0],                    # 1: o'ng-orqa-past
        [x_offset + hw, y_offset + hl, 0],                    # 2: o'ng-old-past
        [x_offset - hw, y_offset + hl, 0],                    # 3: chap-old-past
        [x_offset - hw, y_offset - hl, fence_height],        # 4: chap-orqa-yuqori
        [x_offset + hw, y_offset - hl, fence_height],        # 5: o'ng-orqa-yuqori
        [x_offset + hw, y_offset + hl, fence_height],        # 6: o'ng-old-yuqori
        [x_offset - hw, y_offset + hl, fence_height]         # 7: chap-old-yuqori
    ])
    return vertices

def draw_platform_edges(ax, vertices):
    """Platformaning qirralarini chizish"""
    # Pastki yuz (0-1-2-3-0)
    bottom_face = [0, 1, 2, 3, 0]
    ax.plot(vertices[bottom_face, 0], vertices[bottom_face, 1], vertices[bottom_face, 2], 
            color='gray', linewidth=2, alpha=0.8)
    
    # Yuqori yuz (4-5-6-7-4)
    top_face = [4, 5, 6, 7, 4]
    ax.plot(vertices[top_face, 0], vertices[top_face, 1], vertices[top_face, 2], 
            color='gray', linewidth=2, alpha=0.8)
    
    # Vertikal qirralar
    for i in range(4):
        ax.plot([vertices[i, 0], vertices[i+4, 0]], 
                [vertices[i, 1], vertices[i+4, 1]], 
                [vertices[i, 2], vertices[i+4, 2]], 
                color='gray', linewidth=2, alpha=0.8)
    
    return ax

alpha0 = 0.0
beta0  = 135.0
gamma0 = 15.0
h0     = 30.0
oa0    = 0.0

# Hozirgi qiymatlar (animatsiya uchun)
current_alpha = alpha0
current_beta = beta0
current_gamma = gamma0
current_h = h0
current_oa = oa0

plt.close('all') 

fig = plt.figure(figsize=(14, 9))

ax = fig.add_axes([0.25, 0.05, 0.73, 0.90], projection='3d')
ax.set_facecolor('#f7f7f7')

axis_len = 100
ax.plot([0, axis_len], [0, 0], [0, 0], color='red',   linewidth=1)
ax.plot([0, 0], [0, axis_len], [0, 0], color='green', linewidth=1)
ax.plot([0, 0], [0, 0], [0, axis_len], color='blue',  linewidth=1)

A, B, C, D, E = compute_chain(alpha0, beta0, gamma0, h0, oa0)

# Platformani chizish
platform_vertices = create_platform_vertices(oa0)
platform_lines = []

# Pastki yuz
bottom_face = [0, 1, 2, 3, 0]
line, = ax.plot(platform_vertices[bottom_face, 0], platform_vertices[bottom_face, 1], 
                platform_vertices[bottom_face, 2], color='gray', linewidth=2, alpha=0.8)
platform_lines.append(line)

# Yuqori yuz
top_face = [4, 5, 6, 7, 4]
line, = ax.plot(platform_vertices[top_face, 0], platform_vertices[top_face, 1], 
                platform_vertices[top_face, 2], color='gray', linewidth=2, alpha=0.8)
platform_lines.append(line)

# Vertikal qirralar
for i in range(4):
    line, = ax.plot([platform_vertices[i, 0], platform_vertices[i+4, 0]], 
                    [platform_vertices[i, 1], platform_vertices[i+4, 1]], 
                    [platform_vertices[i, 2], platform_vertices[i+4, 2]], 
                    color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

# Ogorodlarni chizish (ikki yonda, bir joyda turadi)
fence_lines = []

# Y o'qining manfiy tomonidagi ogorod (bir joyda turadi)
fence1_vertices = create_fence_vertices(x_offset=70, y_offset=-50)
for face_indices in [[0, 1, 2, 3, 0], [4, 5, 6, 7, 4]]:  # pastki va yuqori yuzlar
    line, = ax.plot(fence1_vertices[face_indices, 0], fence1_vertices[face_indices, 1], 
                    fence1_vertices[face_indices, 2], color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

for i in range(4):  # vertikal qirralar
    line, = ax.plot([fence1_vertices[i, 0], fence1_vertices[i+4, 0]], 
                    [fence1_vertices[i, 1], fence1_vertices[i+4, 1]], 
                    [fence1_vertices[i, 2], fence1_vertices[i+4, 2]], 
                    color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

# Y o'qining musbat tomonidagi ogorod (bir joyda turadi)
fence2_vertices = create_fence_vertices(x_offset=70, y_offset=50)
for face_indices in [[0, 1, 2, 3, 0], [4, 5, 6, 7, 4]]:  # pastki va yuqori yuzlar
    line, = ax.plot(fence2_vertices[face_indices, 0], fence2_vertices[face_indices, 1], 
                    fence2_vertices[face_indices, 2], color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

for i in range(4):  # vertikal qirralar
    line, = ax.plot([fence2_vertices[i, 0], fence2_vertices[i+4, 0]], 
                    [fence2_vertices[i, 1], fence2_vertices[i+4, 1]], 
                    [fence2_vertices[i, 2], fence2_vertices[i+4, 2]], 
                    color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

sb_line, = ax.plot(
    [A[0], C[0]],
    [A[1], C[1]],
    [A[2], C[2]],
    color='black', linewidth=3, label='BC'
)

cd_line, = ax.plot(
    [C[0], D[0]],
    [C[1], D[1]],
    [C[2], D[2]],
    color='tab:orange', linewidth=3, marker='o', label='CD'
)

de_line, = ax.plot(
    [D[0], E[0]],
    [D[1], E[1]],
    [D[2], E[2]],
    color='tab:blue', linewidth=3, marker='o', label='DE'
)

end_scatter = ax.scatter(E[0], E[1], E[2], color='purple', s=80, marker='*')

# X o'qini -25 dan boshlatish, musbat tomonni ko'proq ko'rsatish
ax.set_xlim(-25, 175)  # X o'qi: -25 dan 150 gacha (musbat tomoni kattaroq)
ax.set_ylim(-axis_len, axis_len)
ax.set_zlim(-10, 190)  # Z o'qini ham biroz yuqoriroq boshlatish
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.view_init(elev=20, azim=-75)
ax.legend(loc='upper left')
ax.set_title(f"3D Визуализация Манипулятора")

info_text = f"α = {alpha0:.1f}°\n\nβ = {beta0:.1f}°\n\nγ = {gamma0:.1f}°\n\nh_CB = {h0:.1f}мм\n\nOA = {oa0:.1f}мм"
info_ax = fig.add_axes([0.08, 0.35, 0.15, 0.20])
info_ax.text(0.1, 0.5, info_text, fontsize=13, verticalalignment='center', fontweight='bold')
info_ax.set_xlim(0, 1)
info_ax.set_ylim(0, 1)
info_ax.axis('off')

alpha_ax = plt.axes([0.12, 0.85, 0.08, 0.04])
beta_ax  = plt.axes([0.12, 0.80, 0.08, 0.04])
gamma_ax = plt.axes([0.12, 0.75, 0.08, 0.04])
h_ax     = plt.axes([0.16, 0.70, 0.08, 0.04])
oa_ax    = plt.axes([0.15, 0.65, 0.08, 0.04])

button_ax = plt.axes([0.08, 0.58, 0.12, 0.05])
animate_button = Button(button_ax, 'ЗАПУСК', color='lightgreen', hovercolor='green')

fig.text(0.08, 0.86, 'α (°):', fontsize=12, fontweight='bold')
fig.text(0.08, 0.81, 'β (°):', fontsize=12, fontweight='bold')
fig.text(0.08, 0.76, 'γ (°):', fontsize=12, fontweight='bold')
fig.text(0.08, 0.71, 'h_CB (мм):', fontsize=12, fontweight='bold')
fig.text(0.08, 0.66, 'OA (мм):', fontsize=12, fontweight='bold')

alpha_textbox = TextBox(alpha_ax, '', initial=str(alpha0))
beta_textbox  = TextBox(beta_ax,  '', initial=str(beta0))
gamma_textbox = TextBox(gamma_ax, '', initial=str(gamma0))
h_textbox     = TextBox(h_ax,     '', initial=str(h0))
oa_textbox    = TextBox(oa_ax,    '', initial=str(oa0))

def start_animation(event):
    try:
        target_alpha = float(alpha_textbox.text)
        target_beta = float(beta_textbox.text)
        target_gamma = float(gamma_textbox.text)
        target_h = float(h_textbox.text)
        target_oa = float(oa_textbox.text)
        
        animate_to_values(target_alpha, target_beta, target_gamma, target_h, target_oa)
    except ValueError:
        print("Ошибка: Проверьте правильность введенных значений!")

def animate_to_values(target_alpha, target_beta, target_gamma, target_h, target_oa):
    global current_alpha, current_beta, current_gamma, current_h, current_oa
    
    steps = 40  # Animatsiya qadamlari soni
    
    # Har bir qadam uchun o'zgarish miqdori
    alpha_step = (target_alpha - current_alpha) / steps
    beta_step = (target_beta - current_beta) / steps
    gamma_step = (target_gamma - current_gamma) / steps
    h_step = (target_h - current_h) / steps
    oa_step = (target_oa - current_oa) / steps
    
    for i in range(steps):
        current_alpha += alpha_step
        current_beta += beta_step
        current_gamma += gamma_step
        current_h += h_step
        current_oa += oa_step
        
        # Grafik yangilash
        A, B, C, D, E = compute_chain(current_alpha, current_beta, current_gamma, current_h, current_oa)
        
        # Platformani yangilash (manipulyator bilan birga harakat qiladi)
        platform_vertices = create_platform_vertices(current_oa)
        
        # Pastki yuz yangilash
        bottom_face = [0, 1, 2, 3, 0]
        platform_lines[0].set_data(platform_vertices[bottom_face, 0], platform_vertices[bottom_face, 1])
        platform_lines[0].set_3d_properties(platform_vertices[bottom_face, 2])
        
        # Yuqori yuz yangilash
        top_face = [4, 5, 6, 7, 4]
        platform_lines[1].set_data(platform_vertices[top_face, 0], platform_vertices[top_face, 1])
        platform_lines[1].set_3d_properties(platform_vertices[top_face, 2])
        
        # Vertikal qirralarni yangilash
        for i in range(4):
            platform_lines[i+2].set_data([platform_vertices[i, 0], platform_vertices[i+4, 0]], 
                                        [platform_vertices[i, 1], platform_vertices[i+4, 1]])
            platform_lines[i+2].set_3d_properties([platform_vertices[i, 2], platform_vertices[i+4, 2]])
        
        # OGORODLAR YANGILANMAYDI - ular bir joyda turadi!
        # (fence_lines o'zgartirilmaydi)
        
        sb_line.set_data([A[0], C[0]], [A[1], C[1]])
        sb_line.set_3d_properties([A[2], C[2]])
        
        cd_line.set_data([C[0], D[0]], [C[1], D[1]])
        cd_line.set_3d_properties([C[2], D[2]])
        
        de_line.set_data([D[0], E[0]], [D[1], E[1]])
        de_line.set_3d_properties([D[2], E[2]])
        
        end_scatter._offsets3d = ([E[0]], [E[1]], [E[2]])
        
        # Ma'lumot panelini yangilash
        info_text = f"α = {current_alpha:.1f}°\n\nβ = {current_beta:.1f}°\n\nγ = {current_gamma:.1f}°\n\nh_CB = {current_h:.1f}мм\n\nOA = {current_oa:.1f}мм"
        info_ax.clear()
        info_ax.text(0.1, 0.5, info_text, fontsize=13, verticalalignment='center', fontweight='bold')
        info_ax.set_xlim(0, 1)
        info_ax.set_ylim(0, 1)
        info_ax.axis('off')
        
        fig.canvas.draw()
        fig.canvas.flush_events()
    
    # Aniq qiymatlarni o'rnatish
    current_alpha = target_alpha
    current_beta = target_beta
    current_gamma = target_gamma
    current_h = target_h
    current_oa = target_oa
# TextBox widgetlari avtomatik ravishda qiymatlarni saqlaydi

# Tugma event handler
animate_button.on_clicked(start_animation)

plt.show()
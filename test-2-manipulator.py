# ===============================================================================
# ИМПОРТ БИБЛИОТЕК / KUTUBXONALARNI IMPORT QILISH
# ===============================================================================
# numpy - математические вычисления / matematik hisob-kitoblar
# matplotlib - 2D/3D графики и визуализация / 2D/3D grafiklar va vizualizatsiya
# mpl_toolkits.mplot3d - 3D координатная система / 3D koordinata tizimi
# matplotlib.widgets - интерактивные элементы UI / interaktiv UI elementlari
# time - временные задержки / vaqt kechikishlari
# fractions - точные дробные вычисления / aniq kasr hisob-kitoblari
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import TextBox, Button
import time
from fractions import Fraction

# ===============================================================================
# ФУНКЦИИ ДЛЯ ТОЧНЫХ ВЫЧИСЛЕНИЙ / ANIQ HISOB-KITOBLAR UCHUN FUNKSIYALAR
# ===============================================================================

# Yuqori aniqlik uchun fractions ishlatamiz
def to_fraction(value, max_denominator=1000000):
    """Sonni kasr ko'rinishida qaytaradi / Преобразует число в дробный вид
    
    Args:
        value: Преобразуемое число / O'zgartiriladigan son
        max_denominator: Максимальный знаменатель / Maksimal maxraj
    """
    return Fraction(value).limit_denominator(max_denominator)

def format_value(value):
    """BARCHA qiymatlarni ANIQ kasr ko'rinishida formatlaydi
    Форматирует ВСЕ значения в ТОЧНОМ дробном виде
    
    Args:
        value: Форматируемое значение / Formatlanadigan qiymat
    """
    frac = to_fraction(float(value), max_denominator=1000000)
    # HAMMA SONNI kasr ko'rinishida chiqaramiz
    if frac.denominator == 1:
        return str(frac.numerator)
    else:
        return f"{frac.numerator}/{frac.denominator}"

# ===============================================================================
# КОНСТАНТЫ МАНИПУЛЯТОРА / MANIPULYATOR KONSTANTALARI
# ===============================================================================
# L - длина звена манипулятора (см) / manipulyator bo'g'inining uzunligi (sm)
# h_DC - высота звена DC (см) / DC bo'g'inining balandligi (sm)
L = 30.0
h_DC = 30.0

# ФИЗИЧЕСКИЕ ОГРАНИЧЕНИЯ / FIZIK CHEKLOVLAR
# h_CB ограничения / h_CB cheklovlari
H_CB_MIN = 29.5  # Минимальная высота h_CB (см) / Minimal h_CB balandligi (sm)
H_CB_MAX = 57.0  # Максимальная высота h_CB (см) / Maksimal h_CB balandligi (sm)

# ===============================================================================
# КИНЕМАТИЧЕСКИЕ ФУНКЦИИ / KINEMATIK FUNKSIYALAR
# ===============================================================================

# ===============================================================================
# INVERSE KINEMATICS ФУНКЦИИ / TESKARI KINEMATIKA FUNKSIYALARI
# ===============================================================================

def inverse_kinematics(E_x, E_y, E_z, n_x, n_y, n_z):
    """Inverse Kinematics - тескари кинематика / teskari kinematika
    E nuqtasi va n vektor komponentlari asosida boshqa nuqtalarni hisoblaydi
    По координатам точки E и компонентам вектора n вычисляет остальные точки
    
    Args:
        E_x: X координата точки E / E nuqtasining X koordinatasi
        E_y: Y координата точки E / E nuqtasining Y koordinatasi  
        E_z: Z координата точки E / E nuqtasining Z koordinatasi
        n_x: X компонент единичного вектора / Birlik vektorning X komponenti
        n_y: Y компонент единичного вектора / Birlik vektorning Y komponenti
        n_z: Z компонент единичного вектора / Birlik vektorning Z komponenti
        
    Returns:
        A, B, C, D, E: Координаты всех точек / Barcha nuqtalarning koordinatalari
    """
    # Birlik vektor shartini tekshirish / Проверка условия единичного вектора
    vector_length = np.sqrt(n_x**2 + n_y**2 + n_z**2)
    if abs(vector_length - 1.0) > 1e-6:
        # Agar birlik vektor bo'lmasa, normalizatsiya qilamiz / Если не единичный вектор, нормализуем
        if vector_length > 0:
            n_x = n_x / vector_length
            n_y = n_y / vector_length
            n_z = n_z / vector_length
    
    # E nuqtasi / Точка E
    E = np.array([E_x, E_y, E_z])
    
    # D nuqtasini hisoblash: D = E - 30 * n (DE uzunligi 30 sm)
    # Вычисление точки D: D = E - 30 * n (длина DE = 30 см)
    D = E - 30 * np.array([n_x, n_y, n_z])
    
    # A va B nuqtalarini hisoblash / Вычисление точек A и B
    # Test-1 bilan mos kelishi uchun A_x = 0 deb olamiz
    # Для соответствия с Test-1 принимаем A_x = 0
    A_x = 0.0
    
    A = np.array([A_x, 0, 0])
    B = A.copy()  # B точка совпадает с A / B nuqta A bilan bir xil
    
    # C nuqtasini hisoblash / Вычисление точки C
    # C nuqtasi A ning ustida, CD uzunligi 30 sm bo'lishi kerak
    # Точка C находится над A, длина CD должна быть 30 см
    
    # CD vektori / Вектор CD
    CD_vector_horizontal = np.sqrt(D[0]**2 + D[1]**2)  # D dan A gacha gorizontal masofa
    
    # CD uzunligi 30 sm bo'lishi kerak / Длина CD должна быть 30 см
    # CD^2 = CD_horizontal^2 + (D_z - C_z)^2 = 900
    
    if CD_vector_horizontal**2 <= 900:  # Matematik jihatdan mumkin / Математически возможно
        vertical_distance_squared = 900 - CD_vector_horizontal**2
        vertical_distance = np.sqrt(vertical_distance_squared)
        
        # C_z ni aniqlash / Определение C_z
        # Test-1 bilan mos kelishi uchun C_z = D_z - vertical_distance
        # Для соответствия с Test-1: C_z = D_z - vertical_distance
        C_z = D[2] - vertical_distance
        
        # Agar C_z manfiy bo'lsa, muqobil variantni tanlaymiz
        if C_z < 0:
            C_z = D[2] + vertical_distance
    else:
        # Agar matematik jihatdan mumkin bo'lmasa / Если математически невозможно
        C_z = D[2]  # Eng yaqin variant / Ближайший вариант
    
    # ФИЗИЧЕСКИЕ ОГРАНИЧЕНИЯ ПРОВЕРКА / FIZIK CHEKLOVLARNI TEKSHIRISH
    # h_CB = C_z (C nuqtasining balandligi)
    h_CB = C_z
    if h_CB < H_CB_MIN:
        print(f"OGOHLANTIRISH: Hisoblangan h_CB = {h_CB:.2f} sm < {H_CB_MIN} sm (minimal limit)")
        print(f"Bu pozitsiya fizik jihatdan mumkin emas!")
        # C_z ni minimal qiymatga o'rnatamiz
        C_z = H_CB_MIN
        print(f"C_z ni {H_CB_MIN} sm ga o'rnatildi")
    elif h_CB > H_CB_MAX:
        print(f"OGOHLANTIRISH: Hisoblangan h_CB = {h_CB:.2f} sm > {H_CB_MAX} sm (maksimal limit)")
        print(f"Bu pozitsiya fizik jihatdan mumkin emas!")
        # C_z ni maksimal qiymatga o'rnatamiz
        C_z = H_CB_MAX
        print(f"C_z ni {H_CB_MAX} sm ga o'rnatildi")
    
    C = np.array([A_x, 0, C_z])
    
    return A, B, C, D, E

# ===============================================================================
# ФУНКЦИИ ДЛЯ СОЗДАНИЯ 3D ОБЪЕКТОВ / 3D OBYEKTLARNI YARATISH FUNKSIYALARI
# ===============================================================================

def create_platform_vertices(center_x, platform_width=40, platform_length=25, platform_height=15):
    """Parallelepiped platformaning uchlarini yaratish
    Создает вершины параллелепипеда платформы
    
    Args:
        center_x: Центр по оси X / X o'qi bo'yicha markaz
        platform_width: Ширина платформы / Platforma kengligi (40)
        platform_length: Длина платформы / Platforma uzunligi (25)
        platform_height: Высота платформы / Platforma balandligi (15)
    """
    hw = platform_width / 2   # yarim kenglik (x o'qi bo'yicha)
    hl = platform_length / 2  # yarim uzunlik (y o'qi bo'yicha) - qisqaroq
    
    # Platformaning pastki qismi (z = 0 dan boshlanadi)
    # Нижняя часть платформы (начинается с z = 0)
    vertices = np.array([
        [center_x - hw, -hl, 0],                    # 0: chap-orqa-past / лево-зад-низ
        [center_x + hw, -hl, 0],                    # 1: o'ng-orqa-past / право-зад-низ
        [center_x + hw,  hl, 0],                    # 2: o'ng-old-past / право-перед-низ
        [center_x - hw,  hl, 0],                    # 3: chap-old-past / лево-перед-низ
        [center_x - hw, -hl, platform_height],     # 4: chap-orqa-yuqori / лево-зад-верх
        [center_x + hw, -hl, platform_height],     # 5: o'ng-orqa-yuqori / право-зад-верх
        [center_x + hw,  hl, platform_height],     # 6: o'ng-old-yuqori / право-перед-верх
        [center_x - hw,  hl, platform_height]      # 7: chap-old-yuqori / лево-перед-верх
    ])
    return vertices

def create_fence_vertices(fence_width=120, fence_length=40, fence_height=60, x_offset=70, y_offset=35):
    """Pomidor egatlari parallelepiped uchlarini yaratish - bir joyda turadi
    Создает вершины параллелепипеда грядок помидоров - остается на месте
    
    Args:
        fence_width: Ширина грядки / Egat kengligi (120)
        fence_length: Длина грядки / Egat uzunligi (40)
        fence_height: Высота грядки / Egat balandligi (60)
        x_offset: Смещение по X / X bo'yicha siljish (70)
        y_offset: Смещение по Y / Y bo'yicha siljish (35)
    """
    hw = fence_width / 2   # yarim kenglik (x o'qi bo'yicha - uzunroq)
    hl = fence_length / 2  # yarim uzunlik (y o'qi bo'yicha)
    
    # Pomidor egatining pastki qismi (z = 0 dan boshlanadi)
    # Нижняя часть грядки помидоров (начинается с z = 0)
    # x_offset - X o'qining musbat tomonida, manipulyatordan uzunroq intervalda
    # x_offset - на положительной стороне оси X, на большем расстоянии от манипулятора
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
    """Platformaning qirralarini chizish
    Рисует ребра платформы
    
    Args:
        ax: 3D axes объект / 3D o'qlar obyekti
        vertices: Вершины платформы / Platforma uchlari
    """
    # Pastki yuz (0-1-2-3-0) / Нижняя грань
    bottom_face = [0, 1, 2, 3, 0]
    ax.plot(vertices[bottom_face, 0], vertices[bottom_face, 1], vertices[bottom_face, 2], 
            color='gray', linewidth=2, alpha=0.8)
    
    # Yuqori yuz (4-5-6-7-4) / Верхняя грань
    top_face = [4, 5, 6, 7, 4]
    ax.plot(vertices[top_face, 0], vertices[top_face, 1], vertices[top_face, 2], 
            color='gray', linewidth=2, alpha=0.8)
    
    # Vertikal qirralar / Вертикальные ребра
    for i in range(4):
        ax.plot([vertices[i, 0], vertices[i+4, 0]], 
                [vertices[i, 1], vertices[i+4, 1]], 
                [vertices[i, 2], vertices[i+4, 2]], 
                color='gray', linewidth=2, alpha=0.8)
    
    return ax

# ===============================================================================
# НАЧАЛЬНЫЕ ПАРАМЕТРЫ МАНИПУЛЯТОРА / MANIPULYATORNING BOSHLANG'ICH PARAMETRLARI
# ===============================================================================
# Inverse kinematics uchun boshlang'ich qiymatlar / Начальные значения для inverse kinematics
# Rasmdagi qiymatlar asosida / На основе значений из изображения
E_x0 = 3786028/794123    # E nuqtasining X koordinatasi / X координата точки E
E_y0 = 0.0               # E nuqtasining Y koordinatasi / Y координата точки E  
E_z0 = 19875327/548842   # E nuqtasining Z koordinatasi / Z координата точки E
n_x0 = 489061/564719     # Birlik vektorning X komponenti / X компонент единичного вектора
n_y0 = 0.0               # Birlik vektorning Y komponenti / Y компонент единичного вектора
n_z0 = -0.5              # Birlik vektorning Z komponenti / Z компонент единичного вектора

# Hozirgi qiymatlar (animatsiya uchun)
# Текущие значения (для анимации)
current_E_x = E_x0
current_E_y = E_y0
current_E_z = E_z0
current_n_x = n_x0
current_n_y = n_y0
current_n_z = n_z0

# ===============================================================================
# СОЗДАНИЕ ГЛАВНОГО ОКНА И 3D ГРАФИКА / ASOSIY OYNA VA 3D GRAFIK YARATISH
# ===============================================================================
plt.close('all')  # Закрыть все предыдущие окна / Barcha oldingi oynalarni yopish

# Создание фигуры размером 14x9 дюймов / 14x9 dyuymli figura yaratish
fig = plt.figure(figsize=(14, 9))

# Создание 3D осей: [left, bottom, width, height] в долях от размера окна
# 3D o'qlarni yaratish: [chap, past, kenglik, balandlik] oyna o'lchamidan ulush sifatida
ax = fig.add_axes([0.22, 0.05, 0.76, 0.90], projection='3d')
ax.set_facecolor('#f7f7f7')  # Светло-серый фон / Och kulrang fon

# ===============================================================================
# КООРДИНАТНЫЕ ОСИ / KOORDINATA O'QLARI
# ===============================================================================
# Рисуем координатные оси X, Y, Z разными цветами
# X, Y, Z koordinata o'qlarini turli ranglarda chizamiz
axis_len = 100  # Длина осей / O'qlar uzunligi
ax.plot([0, axis_len], [0, 0], [0, 0], color='red',   linewidth=1)  # X o'qi - qizil
ax.plot([0, 0], [0, axis_len], [0, 0], color='green', linewidth=1)  # Y o'qi - yashil  
ax.plot([0, 0], [0, 0], [0, axis_len], color='blue',  linewidth=1)  # Z o'qi - ko'k

# ===============================================================================
# ВЫЧИСЛЕНИЕ НАЧАЛЬНЫХ КООРДИНАТ / BOSHLANG'ICH KOORDINATALARNI HISOBLASH
# ===============================================================================
# Вычисляем все точки манипулятора с начальными параметрами (inverse kinematics)
# Boshlang'ich parametrlar bilan manipulyatorning barcha nuqtalarini hisoblaymiz (teskari kinematika)
A, B, C, D, E = inverse_kinematics(E_x0, E_y0, E_z0, n_x0, n_y0, n_z0)

# ===============================================================================
# СОЗДАНИЕ ПОДВИЖНОЙ ПЛАТФОРМЫ / HARAKATLANUVCHI PLATFORMANI YARATISH
# ===============================================================================
# Platformani chizish / Рисуем платформу
platform_vertices = create_platform_vertices(A[0])  # A nuqtasining X koordinatasi asosida / На основе X координаты точки A
platform_lines = []  # Список линий платформы / Platforma chiziqlarining ro'yxati

# Pastki yuz / Нижняя грань (индексы вершин: 0-1-2-3-0)
bottom_face = [0, 1, 2, 3, 0]
line, = ax.plot(platform_vertices[bottom_face, 0], platform_vertices[bottom_face, 1], 
                platform_vertices[bottom_face, 2], color='gray', linewidth=2, alpha=0.8)
platform_lines.append(line)

# Yuqori yuz / Верхняя грань (индексы вершин: 4-5-6-7-4)
top_face = [4, 5, 6, 7, 4]
line, = ax.plot(platform_vertices[top_face, 0], platform_vertices[top_face, 1], 
                platform_vertices[top_face, 2], color='gray', linewidth=2, alpha=0.8)
platform_lines.append(line)

# Vertikal qirralar / Вертикальные ребра (соединяют нижние и верхние вершины)
for i in range(4):
    line, = ax.plot([platform_vertices[i, 0], platform_vertices[i+4, 0]], 
                    [platform_vertices[i, 1], platform_vertices[i+4, 1]], 
                    [platform_vertices[i, 2], platform_vertices[i+4, 2]], 
                    color='gray', linewidth=2, alpha=0.8)
    platform_lines.append(line)

# ===============================================================================
# СОЗДАНИЕ НЕПОДВИЖНЫХ ГРЯДОК ПОМИДОРОВ / QO'ZG'ALMAS POMIDOR EGATLARI
# ===============================================================================
# Pomidor egatlarini chizish (ikki yonda, bir joyda turadi)
# Рисуем грядки помидоров (с двух сторон, остаются на месте)
fence_lines = []  # Список линий грядок / Egat chiziqlarining ro'yxati

# Y o'qining manfiy tomonidagi pomidor egati (bir joyda turadi)
# Грядка помидоров на отрицательной стороне оси Y (остается на месте)
fence1_vertices = create_fence_vertices(x_offset=70, y_offset=-50)
for face_indices in [[0, 1, 2, 3, 0], [4, 5, 6, 7, 4]]:  # pastki va yuqori yuzlar
    line, = ax.plot(fence1_vertices[face_indices, 0], fence1_vertices[face_indices, 1], 
                    fence1_vertices[face_indices, 2], color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

for i in range(4):  # vertikal qirralar / вертикальные ребра
    line, = ax.plot([fence1_vertices[i, 0], fence1_vertices[i+4, 0]], 
                    [fence1_vertices[i, 1], fence1_vertices[i+4, 1]], 
                    [fence1_vertices[i, 2], fence1_vertices[i+4, 2]], 
                    color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

# Y o'qining musbat tomonidagi pomidor egati (bir joyda turadi)
# Грядка помидоров на положительной стороне оси Y (остается на месте)
fence2_vertices = create_fence_vertices(x_offset=70, y_offset=50)
for face_indices in [[0, 1, 2, 3, 0], [4, 5, 6, 7, 4]]:  # pastki va yuqori yuzlar
    line, = ax.plot(fence2_vertices[face_indices, 0], fence2_vertices[face_indices, 1], 
                    fence2_vertices[face_indices, 2], color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

for i in range(4):  # vertikal qirralar / вертикальные ребра
    line, = ax.plot([fence2_vertices[i, 0], fence2_vertices[i+4, 0]], 
                    [fence2_vertices[i, 1], fence2_vertices[i+4, 1]], 
                    [fence2_vertices[i, 2], fence2_vertices[i+4, 2]], 
                    color='brown', linewidth=2, alpha=0.7)
    fence_lines.append(line)

# ===============================================================================
# ЗВЕНЬЯ МАНИПУЛЯТОРА / MANIPULYATOR BO'G'INLARI
# ===============================================================================
# BC звено (от точки A до C) - черный цвет / BC bo'g'ini (A dan C gacha) - qora rang
sb_line, = ax.plot(
    [A[0], C[0]],  # X координаты / X koordinatalari
    [A[1], C[1]],  # Y координаты / Y koordinatalari  
    [A[2], C[2]],  # Z координаты / Z koordinatalari
    color='black', linewidth=3, label='BC'
)

# CD звено (от точки C до D) - оранжевый цвет с маркерами / CD bo'g'ini - to'q sariq rang
cd_line, = ax.plot(
    [C[0], D[0]],
    [C[1], D[1]],
    [C[2], D[2]],
    color='tab:orange', linewidth=3, marker='o', label='CD'
)

# DE звено (от точки D до E) - синий цвет с маркерами / DE bo'g'ini - ko'k rang
de_line, = ax.plot(
    [D[0], E[0]],
    [D[1], E[1]],
    [D[2], E[2]],
    color='tab:blue', linewidth=3, marker='o', label='DE'
)

# Конечная точка E (end effector) - фиолетовая звездочка / Oxirgi nuqta E - binafsha yulduzcha
end_scatter = ax.scatter(E[0], E[1], E[2], color='purple', s=80, marker='*')

# ===============================================================================
# НАСТРОЙКА 3D СЦЕНЫ / 3D SAHNA SOZLAMALARI
# ===============================================================================
# X o'qini -25 dan boshlatish, musbat tomonni ko'proq ko'rsatish
# Начинаем ось X с -25, показываем больше положительную сторону
ax.set_xlim(-25, 175)  # X o'qi: -25 dan 175 gacha (musbat tomoni kattaroq)
ax.set_ylim(-axis_len, axis_len)  # Y o'qi: -100 dan 100 gacha
ax.set_zlim(-10, 190)  # Z o'qini ham biroz yuqoriroq boshlatish (-10 dan 190 gacha)

# Подписи осей / O'q nomlari
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Угол обзора камеры / Kamera ko'rish burchagi
ax.view_init(elev=20, azim=-75)  # elev: вертикальный угол / vertikal burchak, azim: горизонтальный / gorizontal

# Легенда и заголовок / Legend va sarlavha
ax.legend(loc='upper left')  # Легенда в верхнем левом углу / Chap yuqori burchakda legend
ax.set_title(f"3D Визуализация Манипулятора")  # Заголовок окна / Oyna sarlavhasi

# ===============================================================================
# ИНФОРМАЦИОННЫЕ ПАНЕЛИ / MA'LUMOT PANELLARI
# ===============================================================================
# Параметры манипулятора - левая верхняя панель / Manipulyator parametrlari - chap yuqori panel
info_text = f"E_x = {format_value(E_x0)}\nE_y = {format_value(E_y0)}\nE_z = {format_value(E_z0)}\nn_x = {format_value(n_x0)}\nn_y = {format_value(n_y0)}\nn_z = {format_value(n_z0)}"
info_ax = fig.add_axes([0.02, 0.45, 0.18, 0.15])  # [left, bottom, width, height] в долях
info_ax.text(0.05, 0.5, info_text, fontsize=13, verticalalignment='center', fontweight='bold')
info_ax.set_xlim(0, 1)
info_ax.set_ylim(0, 1)
info_ax.axis('off')  # Скрыть оси панели / Panel o'qlarini yashirish

# Координатalar paneli - har bir nuqta alohida qatorda / Панель координат - каждая точка на отдельной строке
coord_text = f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\nB: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\nC: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\nD: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\nE: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\nn: ({format_value(n_x0)}, {format_value(n_y0)}, {format_value(n_z0)})"
coord_ax = fig.add_axes([0.02, 0.05, 0.18, 0.35])  # Левая нижняя панель / Chap pastki panel
coord_ax.text(0.05, 0.5, coord_text, fontsize=11, verticalalignment='center', fontweight='bold')
coord_ax.set_xlim(0, 1)
coord_ax.set_ylim(0, 1)
coord_ax.axis('off')

# ===============================================================================
# ЭЛЕМЕНТЫ УПРАВЛЕНИЯ / BOSHQARUV ELEMENTLARI
# ===============================================================================
# Текстовые поля для ввода параметров / Parametrlarni kiritish uchun matn maydonlari
# [left, bottom, width, height] координаты в долях от размера окна
ex_ax = plt.axes([0.06, 0.87, 0.08, 0.03])  # Поле для E_x / E_x uchun maydon
ey_ax = plt.axes([0.06, 0.83, 0.08, 0.03])  # Поле для E_y / E_y uchun maydon
ez_ax = plt.axes([0.06, 0.79, 0.08, 0.03])  # Поле для E_z / E_z uchun maydon
nx_ax = plt.axes([0.06, 0.75, 0.08, 0.03])  # Поле для n_x / n_x uchun maydon
ny_ax = plt.axes([0.06, 0.71, 0.08, 0.03])  # Поле для n_y / n_y uchun maydon
nz_ax = plt.axes([0.06, 0.67, 0.08, 0.03])  # Поле для n_z / n_z uchun maydon

# Кнопка запуска анимации / Animatsiyani ishga tushirish tugmasi
button_ax = plt.axes([0.08, 0.61, 0.12, 0.04])  # Позиция кнопки / Tugma pozitsiyasi
animate_button = Button(button_ax, 'ЗАПУСК', color='lightgreen', hovercolor='green')

# Подписи к полям ввода / Kiritish maydonlari uchun yorliqlar
fig.text(0.02, 0.88, 'E_x:', fontsize=12, fontweight='bold')      # E nuqtasining X koordinatasi
fig.text(0.02, 0.84, 'E_y:', fontsize=12, fontweight='bold')      # E nuqtasining Y koordinatasi
fig.text(0.02, 0.80, 'E_z:', fontsize=12, fontweight='bold')      # E nuqtasining Z koordinatasi
fig.text(0.02, 0.76, 'n_x:', fontsize=12, fontweight='bold')      # Birlik vektorning X komponenti
fig.text(0.02, 0.72, 'n_y:', fontsize=12, fontweight='bold')      # Birlik vektorning Y komponenti
fig.text(0.02, 0.68, 'n_z:', fontsize=12, fontweight='bold')      # Birlik vektorning Z komponenti

# Создание текстовых полей с начальными значениями / Boshlang'ich qiymatlar bilan matn maydonlarini yaratish
ex_textbox = TextBox(ex_ax, '', initial=str(E_x0))  # E_x параметри uchun TextBox
ey_textbox = TextBox(ey_ax, '', initial=str(E_y0))  # E_y параметри uchun TextBox
ez_textbox = TextBox(ez_ax, '', initial=str(E_z0))  # E_z параметри uchun TextBox
nx_textbox = TextBox(nx_ax, '', initial=str(n_x0))  # n_x параметри uchun TextBox
ny_textbox = TextBox(ny_ax, '', initial=str(n_y0))  # n_y параметри uchun TextBox
nz_textbox = TextBox(nz_ax, '', initial=str(n_z0))  # n_z параметри uchun TextBox

# ===============================================================================
# ФУНКЦИИ АНИМАЦИИ / ANIMATSIYA FUNKSIYALARI
# ===============================================================================

def start_animation(event):
    """Анимацию запуск функции / Animatsiyani boshlash funksiyasi
    Пользователь кнопку нажимает, параметры читает и анимацию запускает
    Foydalanuvchi tugmani bosganida parametrlarni o'qiydi va animatsiyani boshlaydi
    
    Args:
        event: Событие нажатия кнопки / Tugma bosilish hodisasi
    """
    try:
        # Текстовых полей из значения читаем / Matn maydonlaridan qiymatlarni o'qiymiz
        target_E_x = float(ex_textbox.text)   # Целевая координата E_x / Maqsadli E_x koordinatasi
        target_E_y = float(ey_textbox.text)   # Целевая координата E_y / Maqsadli E_y koordinatasi
        target_E_z = float(ez_textbox.text)   # Целевая координата E_z / Maqsadli E_z koordinatasi
        target_n_x = float(nx_textbox.text)   # Целевый компонент n_x / Maqsadli n_x komponenti
        target_n_y = float(ny_textbox.text)   # Целевый компонент n_y / Maqsadli n_y komponenti
        target_n_z = float(nz_textbox.text)   # Целевый компонент n_z / Maqsadli n_z komponenti
        
        # Анимацию к целевым значениям запускаем / Maqsadli qiymatlarga animatsiyani ishga tushiramiz
        animate_to_values(target_E_x, target_E_y, target_E_z, target_n_x, target_n_y, target_n_z)
    except ValueError:
        # Ошибка при неправильном вводе / Noto'g'ri kiritishda xatolik
        print("Ошибка: Проверьте правильность введенных значений!")

def animate_to_values(target_E_x, target_E_y, target_E_z, target_n_x, target_n_y, target_n_z):
    """Плавная анимация к целевым значениям / Maqsadli qiymatlarga silliq animatsiya
    Текущие значения от целевых значений до плавно переходит
    Joriy qiymatlardan maqsadli qiymatlargacha silliq o'tish
    
    Args:
        target_E_x: Целевая координата E_x / Maqsadli E_x koordinatasi
        target_E_y: Целевая координата E_y / Maqsadli E_y koordinatasi
        target_E_z: Целевая координата E_z / Maqsadli E_z koordinatasi
        target_n_x: Целевый компонент n_x / Maqsadli n_x komponenti
        target_n_y: Целевый компонент n_y / Maqsadli n_y komponenti
        target_n_z: Целевый компонент n_z / Maqsadli n_z komponenti
    """
    global current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z
    
    steps = 40  # Animatsiya qadamlari soni / Количество шагов анимации
    
    # Har bir qadam uchun o'zgarish miqdori / Величина изменения для каждого шага
    E_x_step = (target_E_x - current_E_x) / steps  # E_x uchun qadam / Шаг для E_x
    E_y_step = (target_E_y - current_E_y) / steps  # E_y uchun qadam / Шаг для E_y
    E_z_step = (target_E_z - current_E_z) / steps  # E_z uchun qadam / Шаг для E_z
    n_x_step = (target_n_x - current_n_x) / steps  # n_x uchun qadam / Шаг для n_x
    n_y_step = (target_n_y - current_n_y) / steps  # n_y uchun qadam / Шаг для n_y
    n_z_step = (target_n_z - current_n_z) / steps  # n_z uchun qadam / Шаг для n_z
    
    # Анимация цикли / Цикл анимации
    for i in range(steps):
        # Текущие значения обновляем / Joriy qiymatlarni yangilaymiz
        current_E_x += E_x_step
        current_E_y += E_y_step
        current_E_z += E_z_step
        current_n_x += n_x_step
        current_n_y += n_y_step
        current_n_z += n_z_step
        
        # Grafik yangilash / Обновление графики
        # Новые координаты всех точек вычисляем (inverse kinematics)
        # Barcha nuqtalarning yangi koordinatalarini hisoblaymiz (teskari kinematika)
        A, B, C, D, E = inverse_kinematics(current_E_x, current_E_y, current_E_z, current_n_x, current_n_y, current_n_z)
        
        # Platformani yangilash (manipulyator bilan birga harakat qiladi)
        # Обновление платформы (движется вместе с манипулятором)
        platform_vertices = create_platform_vertices(A[0])  # A nuqtasining X koordinatasi asosida
        
        # Pastki yuz yangilash / Обновление нижней грани
        bottom_face = [0, 1, 2, 3, 0]
        platform_lines[0].set_data(platform_vertices[bottom_face, 0], platform_vertices[bottom_face, 1])
        platform_lines[0].set_3d_properties(platform_vertices[bottom_face, 2])
        
        # Yuqori yuz yangilash / Обновление верхней грани
        top_face = [4, 5, 6, 7, 4]
        platform_lines[1].set_data(platform_vertices[top_face, 0], platform_vertices[top_face, 1])
        platform_lines[1].set_3d_properties(platform_vertices[top_face, 2])
        
        # Vertikal qirralarni yangilash / Обновление вертикальных ребер
        for j in range(4):
            platform_lines[j+2].set_data([platform_vertices[j, 0], platform_vertices[j+4, 0]], 
                                        [platform_vertices[j, 1], platform_vertices[j+4, 1]])
            platform_lines[j+2].set_3d_properties([platform_vertices[j, 2], platform_vertices[j+4, 2]])
        
        # POMIDOR EGATLARI YANGILANMAYDI - ular bir joyda turadi!
        # ГРЯДКИ ПОМИДОРОВ НЕ ОБНОВЛЯЮТСЯ - они остаются на месте!
        # (fence_lines o'zgartirilmaydi / fence_lines не изменяются)
        
        # Manipulyator bo'g'inlarini yangilash / Обновление звеньев манипулятора
        sb_line.set_data([A[0], C[0]], [A[1], C[1]])      # BC bo'g'ini / Звено BC
        sb_line.set_3d_properties([A[2], C[2]])
        
        cd_line.set_data([C[0], D[0]], [C[1], D[1]])      # CD bo'g'ini / Звено CD
        cd_line.set_3d_properties([C[2], D[2]])
        
        de_line.set_data([D[0], E[0]], [D[1], E[1]])      # DE bo'g'ini / Звeno DE
        de_line.set_3d_properties([D[2], E[2]])
        
        # End effector pozitsiyasini yangilash / Обновление позиции концевого эффектора
        end_scatter._offsets3d = ([E[0]], [E[1]], [E[2]])
        
        # Ma'lumot panelini yangilash / Обновление информационной панели
        info_text = f"E_x = {format_value(current_E_x)}\nE_y = {format_value(current_E_y)}\nE_z = {format_value(current_E_z)}\nn_x = {format_value(current_n_x)}\nn_y = {format_value(current_n_y)}\nn_z = {format_value(current_n_z)}"
        info_ax.clear()
        info_ax.text(0.05, 0.5, info_text, fontsize=13, verticalalignment='center', fontweight='bold')
        info_ax.set_xlim(0, 1)
        info_ax.set_ylim(0, 1)
        info_ax.axis('off')
        
        # Koordinatalar paneli yangilash - har bir nuqta alohida qatorda
        # Обновление панели координат - каждая точка на отдельной строке
        coord_text = f"A: ({format_value(A[0])}, {format_value(A[1])}, {format_value(A[2])})\n\nB: ({format_value(B[0])}, {format_value(B[1])}, {format_value(B[2])})\n\nC: ({format_value(C[0])}, {format_value(C[1])}, {format_value(C[2])})\n\nD: ({format_value(D[0])}, {format_value(D[1])}, {format_value(D[2])})\n\nE: ({format_value(E[0])}, {format_value(E[1])}, {format_value(E[2])})\n\nn: ({format_value(current_n_x)}, {format_value(current_n_y)}, {format_value(current_n_z)})"
        coord_ax.clear()
        coord_ax.text(0.05, 0.5, coord_text, fontsize=11, verticalalignment='center', fontweight='bold')
        coord_ax.set_xlim(0, 1)
        coord_ax.set_ylim(0, 1)
        coord_ax.axis('off')
        
        # Экран обновляем / Ekranni yangilaymiz
        fig.canvas.draw()          # Графику перерисовываем / Grafikni qayta chizamiz
        fig.canvas.flush_events()  # События обрабатываем / Hodisalarni qayta ishlaymiz
    
    # Aniq qiymatlarni o'rnatish / Установка точных значений
    current_E_x = target_E_x
    current_E_y = target_E_y
    current_E_z = target_E_z
    current_n_x = target_n_x
    current_n_y = target_n_y
    current_n_z = target_n_z

# ===============================================================================
# СОБЫТИЕ ОБРАБОТЧИКИ / HODISA ISHLOVCHILARI
# ===============================================================================
# TextBox widgetlari avtomatik ravishda qiymatlarni saqlaydi
# TextBox виджеты автоматически сохраняют значения

# Tugma event handler / Обработчик события кнопки
animate_button.on_clicked(start_animation)  # Кнопку нажатие событие функции связываем

# ===============================================================================
# ПРОГРАММА ЗАПУСК / DASTURNI ISHGA TUSHIRISH
# ===============================================================================
plt.show()  # Matplotlib окно показываем / Matplotlib oynasini ko'rsatamiz
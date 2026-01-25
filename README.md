# Robotics Manipulator Visualization

3D robotik manipulyator vizualizatsiya loyihasi Python va matplotlib yordamida.

## Loyiha haqida

Bu loyiha robotik manipulyatorning 3D vizualizatsiyasini ta'minlaydi va quyidagi imkoniyatlarga ega:

- **3D Manipulyator**: BC, CD, DE bo'g'inlari bilan
- **Interaktiv boshqaruv**: α, β, γ burchaklari va h_CB, OA parametrlarini real vaqtda o'zgartirish
- **Animatsiya**: Silliq o'tish animatsiyalari
- **Platforma**: Manipulyator uchun harakatlanuvchi platforma
- **Ogorodlar**: Ish muhitini taqlid qiluvchi qo'zg'almas to'siqlar

## Xususiyatlar

### Vizual elementlar:
- **Koordinata o'qlari**: X (qizil), Y (yashil), Z (ko'k)
- **Manipulyator bo'g'inlari**: 
  - BC: qora rang
  - CD: to'q sariq rang
  - DE: ko'k rang
- **End effector**: Binafsha yulduzcha
- **Platforma**: Kulrang parallelepiped
- **Ogorodlar**: Jigarrang to'rtburchaklar prizmalari

### Interaktiv boshqaruv:
- α (alpha): Aylanish burchagi
- β (beta): Egilish burchagi  
- γ (gamma): Qo'shimcha burchak
- h_CB: Vertikal masofa
- OA: Gorizontal siljish

## Foydalanish

```bash
python test-1-manipulator.py
```

### Boshqaruv:
1. Chap paneldagi matn maydonlariga yangi qiymatlarni kiriting
2. "ЗАПУСК" tugmasini bosing
3. Animatsiya avtomatik ravishda boshlanadi

## Texnik ma'lumotlar

- **Til**: Python 3.x
- **Kutubxonalar**: 
  - numpy - matematik hisob-kitoblar
  - matplotlib - 3D vizualizatsiya
  - mpl_toolkits.mplot3d - 3D plotting

## Loyiha tuzilishi

```
inverse_kinematics/
├── test-1-manipulator.py    # Asosiy dastur
├── .kiro/                   # Kiro IDE sozlamalari
│   └── steering/
│       └── design-system.md # Dizayn tizimi qoidalari
├── .gitignore              # Git ignore fayli
└── README.md               # Bu fayl
```

## Matematik model

Loyiha forward kinematics (to'g'ri kinematika) algoritmini ishlatadi:

- **L**: Bo'g'in uzunligi (30.0)
- **h_DC**: DC bo'g'in balandligi (30.0)
- **Forward kinematics**: Burchaklar asosida end effector pozitsiyasini hisoblash

## Muallif

Robotik manipulyator vizualizatsiya tizimi

## Litsenziya

Bu loyiha ochiq kodli hisoblanadi.
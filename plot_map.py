import numpy as np
import matplotlib.pyplot as plt

GRID_SIZE = 140
grid = np.full((GRID_SIZE, GRID_SIZE), -1)

with open(r"C:\D\FEI\RMR\uloha3\build\Desktop_Qt_6_10_2_MSVC2022_64bit-Release\demoRMR\map.txt", "r") as f:
    for line in f:
        x, y, occ = map(int, line.split())
        grid[y][x] = occ

img = np.zeros((GRID_SIZE, GRID_SIZE))

for y in range(GRID_SIZE):
    for x in range(GRID_SIZE):
        if grid[y][x] == -1:
            img[y][x] = 0.5
        elif grid[y][x] == 0:
            img[y][x] = 1.0
        elif grid[y][x] == 1:
            img[y][x] = 0.0

img = np.flipud(img)

plt.imshow(img, cmap="gray")
plt.title("Occupancy Grid")
plt.axis("off")
plt.show()
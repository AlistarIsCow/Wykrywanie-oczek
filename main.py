import skimage.io
from skimage.feature import canny
from skimage.measure import regionprops
from skimage.morphology import label, dilation

import numpy as np
from math import dist
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches


def check_shape(a, b, c, d, edge_tolerance):    #sprawdzanie, czy obiekt jest mniej wiecej kwadratem (w tym kołem)
    distance_ab = dist(a, b)  #gora
    distance_bc = dist(b, c)  #prawo
    distance_cd = dist(c, d)  #dol
    distance_da = dist(d, a)  #lewo
    average_distance = np.average([distance_ab, distance_bc, distance_cd, distance_da])
    if abs(distance_ab - average_distance)/average_distance < edge_tolerance:
        if abs(distance_bc - average_distance) / average_distance < edge_tolerance:
            if abs(distance_cd - average_distance) / average_distance < edge_tolerance:
                if abs(distance_da - average_distance) / average_distance < edge_tolerance:
                    return True
    return False


def check_area(area, expected_area, area_tolerance):
    if abs(area - expected_area) / expected_area < area_tolerance:
        return True
    return False


minr = 0
minc = 0
maxr = 0
maxc = 0
do_we_continue = True
minimum_size = 10
tolerance = 0.17
areas = [0]
items = []
circles = []
is_free = []
ones = []
twos = []
threes = []
fours = []
fives = []
sixes = []

filename = "photos/easy/10.jpg"
image = skimage.io.imread(filename, as_gray=True)
end_image = skimage.io.imread(filename)

edges = canny(image, sigma=3)       #sigma >=3
edges_dilation = dilation(edges)
fig, ax = plt.subplots()
label_image = label(edges_dilation, connectivity=1)

for region in regionprops(label_image):
    minr, minc, maxr, maxc = region.bbox
    vertex_1 = [minc, minr]  # lewy-górny
    vertex_2 = [maxc, minr]  # prawy-górny
    vertex_3 = [maxc, maxr]  # prawy-dolny
    vertex_4 = [minc, maxr]  # lewy-dolny
    if region.area >= minimum_size:
        if check_shape(vertex_1, vertex_2, vertex_3, vertex_4, tolerance):
            areas.append(region.area)
            items.append(region)
            #rect = mpatches.Rectangle((minc, minr), maxc - minc, maxr - minr, fill=False, edgecolor='red', linewidth=2)
            #ax.add_patch(rect)
            #print("area: {}     centroid: {}".format(region.area, region.centroid))
areas = sorted(areas)
average_area = areas[int(len(areas)/2)-1]
print("Average area: {}".format(average_area))

for item in items:
    minr, minc, maxr, maxc = item.bbox
    vertex_1 = [minc, minr]         #lewy-gorny
    vertex_2 = [maxc, minr]         #prawy-gorny
    vertex_3 = [maxc, maxr]         #prawy-dolny
    vertex_4 = [minc, maxr]         #lewy-dolny
    if item.area >= minimum_size:
        if check_area(item.area, average_area, tolerance):
            circles.append(item)
            #rect = mpatches.Rectangle((minc, minr), maxc - minc, maxr - minr, fill=False, edgecolor='red', linewidth=2)
            #ax.add_patch(rect)
            print("area: {}     centroid: {}".format(item.area, item.centroid))


for i in range(len(circles)):
    is_free.append(True)

r = ((maxr-minr)/2 + (maxc-minc)/2)/2
print("Average r: {}".format(r))
#sprawdzamy odleglosci pomiedzy poszczegolnymi oczkami
for i in range(0, len(circles)):        #5
    for j in range(0, len(circles)):
        if dist(circles[i].centroid, circles[j].centroid) < 7*r and do_we_continue and is_free[i] and is_free[j] and i != j:
            for k in range(0, len(circles)):
                if dist(circles[i].centroid, circles[k].centroid) < 7 * r and is_free[k] and k not in [i, j]:
                    for l in range(0, len(circles)):
                        if dist(circles[i].centroid, circles[l].centroid) < 7 * r and is_free[l] and l not in [i, j, k]:
                            for m in range(0, len(circles)):
                                if dist(circles[i].centroid, circles[m].centroid) < 7 * r and is_free[m] and m not in [i, j, k, l]:
                                    if abs((dist(circles[i].centroid, circles[j].centroid)) - 3.91*r)/(3.91*r) < tolerance:
                                        if abs((dist(circles[i].centroid, circles[k].centroid)) - 3.91*r)/(3.91*r) < tolerance:
                                            if abs((dist(circles[j].centroid, circles[l].centroid)) - 3.91*r)/(3.91*r) < tolerance:
                                                if abs((dist(circles[k].centroid, circles[l].centroid)) - 3.91*r)/(3.91*r) < tolerance:
                                                    if abs((dist(circles[i].centroid, circles[m].centroid)) - 2.77*r)/(2.77*r) < tolerance:
                                                        if abs((dist(circles[l].centroid, circles[m].centroid)) - 2.77 * r) / (2.77 * r) < tolerance:
                                                            is_free[i] = False
                                                            is_free[j] = False
                                                            is_free[k] = False
                                                            is_free[l] = False
                                                            is_free[m] = False
                                                            print("5    i: {}    j: {}   k: {}   l: {}   m: {}".format(i, j, k, l, m))
                                                            fives.append([circles[m].centroid])
                                                            if True not in is_free:
                                                                do_we_continue = False

tmp1 = []
tmp2 = []
for i in range(len(is_free)):
    if is_free[i]:
        tmp1.append(True)
        tmp2.append(circles[i])
is_free = list(tmp1)
circles = list(tmp2)

for i in range(0, len(circles)):        #6
    for j in range(0, len(circles)):
        if dist(circles[i].centroid, circles[j].centroid) < 7*r and do_we_continue and is_free[i] and is_free[j] and i != j:
            for k in range(0, len(circles)):
                if dist(circles[i].centroid, circles[k].centroid) < 7 * r and is_free[k] and k not in [i, j]:
                    for l in range(0, len(circles)):
                        if dist(circles[i].centroid, circles[l].centroid) < 7 * r and is_free[l] and l not in [i, j, k]:
                            for m in range(0, len(circles)):
                                if dist(circles[i].centroid, circles[m].centroid) < 7 * r and is_free[m] and m not in [i, j, k, l]:
                                    for n in range(0, len(circles)):
                                        if dist(circles[i].centroid, circles[n].centroid) < 7 * r and is_free[n] and n not in [i, j, k, l, m]:
                                            if abs((dist(circles[i].centroid, circles[j].centroid)) - 3.58*r)/(3.58*r) < tolerance:
                                                if abs((dist(circles[k].centroid, circles[l].centroid)) - 3.58*r)/(3.58*r) < tolerance:
                                                    if abs((dist(circles[m].centroid, circles[n].centroid)) - 3.58*r)/(3.58*r) < tolerance:
                                                        if abs((dist(circles[i].centroid, circles[m].centroid)) - 4.51*r)/(4.51*r) < tolerance:
                                                            if abs((dist(circles[j].centroid, circles[n].centroid)) - 4.51*r)/(4.51*r) < tolerance:
                                                                if abs((dist(circles[i].centroid, circles[n].centroid)) - 5.75 * r) / (5.75 * r) < tolerance:
                                                                    if abs((dist(circles[j].centroid, circles[m].centroid)) - 5.75 * r) / (5.75 * r) < tolerance:
                                                                        if abs((dist(circles[i].centroid, circles[k].centroid)) - 2.25 * r) / (2.25 * r) < tolerance:
                                                                            if abs((dist(circles[k].centroid, circles[m].centroid)) - 2.25 * r) / (2.25 * r) < tolerance:
                                                                                is_free[i] = False
                                                                                is_free[j] = False
                                                                                is_free[k] = False
                                                                                is_free[l] = False
                                                                                is_free[m] = False
                                                                                is_free[n] = False
                                                                                print("6    i: {}    j: {}   k: {}   l: {}   m: {}   n: {}".format(i, j, k, l, m, n))
                                                                                sixes.append([circles[k].centroid, circles[l].centroid])
                                                                                if True not in is_free:
                                                                                    do_we_continue = False

tmp1 = []
tmp2 = []
for i in range(len(is_free)):
    if is_free[i]:
        tmp1.append(True)
        tmp2.append(circles[i])
is_free = list(tmp1)
circles = list(tmp2)

for i in range(0, len(circles)):        #4
    for j in range(0, len(circles)):
        if dist(circles[i].centroid, circles[j].centroid) < 7*r and do_we_continue and is_free[i] and is_free[j] and i != j:
            for k in range(0, len(circles)):
                if dist(circles[i].centroid, circles[k].centroid) < 7 * r and is_free[k] and k not in [i, j]:
                    for l in range(0, len(circles)):
                        if dist(circles[i].centroid, circles[l].centroid) < 7 * r and is_free[l] and l not in [i, j, k]:
                            if abs((dist(circles[i].centroid, circles[j].centroid)) - 3.91*r)/(3.91*r) < tolerance:
                                if abs((dist(circles[i].centroid, circles[k].centroid)) - 3.91*r)/(3.91*r) < tolerance:
                                    if abs((dist(circles[j].centroid, circles[l].centroid)) - 3.91*r)/(3.91*r) < tolerance:
                                        if abs((dist(circles[k].centroid, circles[l].centroid)) - 3.91*r)/(3.91*r) < tolerance:
                                            if abs((dist(circles[i].centroid, circles[l].centroid)) - 5.54*r)/(5.54*r) < tolerance:
                                                if abs((dist(circles[j].centroid, circles[k].centroid)) - 5.54 * r) / (5.54 * r) < tolerance:
                                                    is_free[i] = False
                                                    is_free[j] = False
                                                    is_free[k] = False
                                                    is_free[l] = False
                                                    print("4    i: {}    j: {}   k: {}   l: {}".format(i, j, k, l))
                                                    fours.append([circles[i].centroid, circles[j].centroid, circles[k].centroid, circles[l].centroid])
                                                    if True not in is_free:
                                                        do_we_continue = False


for i in range(0, len(circles)):        #3
    for j in range(0, len(circles)):
        if dist(circles[i].centroid, circles[j].centroid) < 7*r and do_we_continue and is_free[i] and is_free[j] and i != j:
            for k in range(0, len(circles)):
                if dist(circles[i].centroid, circles[k].centroid) < 7 * r and is_free[k] and k not in [i, j]:
                    if abs((dist(circles[i].centroid, circles[j].centroid)) - 2.77*r)/(2.77*r) < tolerance:
                        if abs((dist(circles[j].centroid, circles[k].centroid)) - 2.77*r)/(2.77*r) < tolerance:
                            if abs((dist(circles[i].centroid, circles[k].centroid)) - 5.54*r)/(5.54*r) < tolerance:
                                is_free[i] = False
                                is_free[j] = False
                                is_free[k] = False
                                print("3    i: {}    j: {}   k: {}".format(i, j, k))
                                threes.append([circles[j].centroid])
                                if True not in is_free:
                                    do_we_continue = False


for i in range(0, len(circles)):        #2
    for j in range(0, len(circles)):
        if do_we_continue and is_free[i] and is_free[j]:
            if i != j:
                if is_free[i] and is_free[j]:
                    if abs((dist(circles[i].centroid, circles[j].centroid)) - 5.54*r)/(5.54*r) < tolerance:
                        is_free[i] = False
                        is_free[j] = False
                        print("2    i: {}    j: {}".format(i, j))
                        twos.append([circles[i].centroid, circles[j].centroid])
                        if True not in is_free:
                            do_we_continue = False


for i in range(0, len(circles)):        #1
    if is_free[i]:
        is_free[i] = False
        print("1    i: {}".format(i))
        ones.append([circles[i].centroid])
        if True not in is_free:
            do_we_continue = False

#rysowania kwadratów i cyfr wzgledem srodka kostki
for elem in ones:
    average_x = elem[0][0]
    average_y = elem[0][1]
    rect = mpatches.Rectangle((average_y - 4*r, average_x - 4*r), 8*r, 8*r, fill=False, edgecolor='red', linewidth=2)
    ax.add_patch(rect)
    plt.text(average_y, average_x, "1", size=1.5*r, color="red", ha="center", va="center")

for elem in twos:
    average_x = (elem[0][0] + elem[1][0])/2
    average_y = (elem[0][1] + elem[1][1])/2
    rect = mpatches.Rectangle((average_y - 4*r, average_x - 4*r), 8*r, 8*r, fill=False, edgecolor='red', linewidth=2)
    ax.add_patch(rect)
    plt.text(average_y, average_x, "2", size=1.5*r, color="red", ha="center", va="center")

for elem in threes:
    average_x = elem[0][0]
    average_y = elem[0][1]
    rect = mpatches.Rectangle((average_y - 4*r, average_x - 4*r), 8*r, 8*r, fill=False, edgecolor='red', linewidth=2)
    ax.add_patch(rect)
    plt.text(average_y, average_x, "3", size=1.5*r, color="red", ha="center", va="center")

for elem in fours:
    average_x = (elem[0][0] + elem[1][0] + elem[2][0] + elem[3][0])/4
    average_y = (elem[0][1] + elem[1][1] + elem[2][1] + elem[3][1])/4
    rect = mpatches.Rectangle((average_y - 4*r, average_x - 4*r), 8*r, 8*r, fill=False, edgecolor='red', linewidth=2)
    ax.add_patch(rect)
    plt.text(average_y, average_x, "4", size=1.5*r, color="red", ha="center", va="center")

for elem in fives:
    average_x = elem[0][0]
    average_y = elem[0][1]
    rect = mpatches.Rectangle((average_y - 4*r, average_x - 4*r), 8*r, 8*r, fill=False, edgecolor='red', linewidth=2)
    ax.add_patch(rect)
    plt.text(average_y, average_x, "5", size=1.5*r, color="red", ha="center", va="center")

for elem in sixes:
    average_x = (elem[0][0] + elem[1][0])/2
    average_y = (elem[0][1] + elem[1][1])/2
    rect = mpatches.Rectangle((average_y - 4*r, average_x - 4*r), 8*r, 8*r, fill=False, edgecolor='red', linewidth=2)
    ax.add_patch(rect)
    plt.text(average_y, average_x, "6", size=1.5*r, color="red", ha="center", va="center")

plt.imshow(image, cmap=plt.cm.gray)
#plt.savefig("final_image2.jpg")
plt.show()

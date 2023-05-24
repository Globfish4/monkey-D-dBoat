import matplotlib.pyplot as plt
import numpy as np


x = []
y = []
fichier = open("log.txt", "r")
lignes = fichier.readlines()
for k in lignes:
    if k!="\n":
        k_split=k.split()
        x.append(eval(k_split[0]))
        y.append(eval(k_split[1]))


def convert_coordinates(lat, lon):
        roh = 6371000
        lat = lat*np.pi/180
        lon = lon*np.pi/180
        lat_ponton = np.pi*48.198943/180
        lon_ponton = -np.pi*3.014750/180
        xtilde = roh * np.cos(lat) * (lon - lon_ponton)
        ytilde = roh * (lat - lat_ponton)
        return np.array([[xtilde], [ytilde]]) #retourne un vecteur colonne


p_bfond = convert_coordinates(48.1994, -3.0166)
X, Y = [0, p_bfond[0][0]], [0, p_bfond[1][0]]
plt.plot(X, Y, c='red')

area = [60**2, 60**2]
plt.scatter(X, Y, s=area, alpha=0.5)

if len(x) == len(y):
    plt.plot(x, y)
    plt.show()
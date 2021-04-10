import math
import mathutils as m
from PyKDL import *

def main():
    # wczytanie danych z pliku
    DHtab=[]
    plik = "../urdf/DHtab.txt"
    f = open(plik,"r")
    for line in f:
    	DHtab.append(line.split())
    f.close()
    dlugosci = [0.1, 0.2, 0.1]
    i=0
    plik = "../urdf/parametry.yaml"
    f = open(plik, 'w')
    for linia in DHtab:
    	if linia[0] == 'i':
    		continue
    	#stworzenie link
    	a = float(linia[1])
    	alfa = float(linia[2])
    	d = float(linia[3])
    	theta = float(linia[4])
    	nazwa = linia[5]
    	#liczenie macierzy transformacji jednorodnej
    	rotTheta = m.Matrix.Rotation(theta, 4, 'Z')
    	transZ = m.Matrix.Translation((0,0,d))
    	transX = m.Matrix.Translation((a,0,0))
    	rotAlfa = m.Matrix.Rotation(alfa, 4, 'X')
    	
    	T = rotTheta @ transZ @ transX @ rotAlfa
    	rpy = T.to_euler()
    	xyz = T.to_translation()
    	
    	#zapis
    	f.write(f'{nazwa}:\n')
    	f.write(f'  rpy: {rpy[0]} {rpy[1]} {rpy[2]}\n')
    	f.write(f'  xyz: {xyz[0]} {xyz[1]} {xyz[2]}\n')
    	f.write(f'  dlugosc: {dlugosci[i]}\n')
    	i = i+1
    	#print(f' rpy {rpy} xyz {xyz} \n')
    	
    	
    	
    
    
    
    	

    
    	
    


if __name__ == '__main__':
    main()
import math

def main():
    # wczytanie danych z pliku
    DHtab=[]
    plik = "../urdf/DHtab.txt"
    f = open(plik,"r")
    for line in f:
    	DHtab.append(line.split())
    f.close()
    
    #plik urdf
    f = open("../urdf/robot.urdf.xml","w")
    f.write('<robot name="robot">\n\n')
    f.write('	<link name="baza">\n')
    f.write('		<visual>\n')
    f.write('			<material name="blue">\n')
    f.write('				<color rgba="0 0 0.8 1"/>\n')
    f.write('			</material>\n')
    f.write('			<geometry>\n')
    f.write('				<origin xyz="0 0 0" rpy="0 0 0"/>\n')
    f.write('				<cylinder length="0.5" radius="0.2"/>\n')   
    f.write('			</geometry>\n')
    f.write('		</visual>\n')
    f.write('	</link>\n')
    poczatek="baza"
    
    for linia in DHtab:
    	if linia[0] == 'i':
    		continue
    	#stworzenie link
    	a = linia[1]
    	alfa = linia[2]
    	d = linia[3]
    	theta = linia[4]
    	nazwa = linia[5]
    	f.write(f'	<link name="{nazwa}">\n')
    	f.write(f'		<visual>\n')
    	f.write(f'			<material name="red">\n')
    	f.write(f'				<color rgba="1 0 0 1"/>\n')
    	f.write(f'			</material>\n')
    	f.write(f'			<geometry>\n')
    	f.write(f'				<origin xyz="0 0 0" rpy="0 0 0"/>\n') #????????????
    	f.write(f'				<cylinder length="0.2" radius="0.1"/>\n')  #??????????
    	f.write(f'			</geometry>\n')
    	f.write(f'		</visual>\n')
    	f.write(f'	</link>\n')
    	
    	#stworzenie joint
    	typ = "revolute"
    	f.write(f'	<joint name="poloczenie-{poczatek}-{nazwa}" type="{typ}">\n')
    	f.write(f'		<origin xyz="{a} 0 {d}" rpy="{alfa} 0 0"/>\n') #????????????
    	f.write(f'		<parent link="{poczatek}"/>\n')
    	f.write(f'		<child link="{nazwa}" />\n')
    	f.write(f'		<axis xyz="0 0 1" />\n')
    	f.write(f'		<limit upper="0" lower="-.5" effort="10" velocity="10" />\n')
    	f.write(f'	</joint>\n')
    	poczatek = nazwa
    	
    #stworzenie koncowki
    f.write('	<link name="koncowka">\n')
    f.write('		<visual>\n')
    f.write('			<material name="blue">\n')
    f.write('				<color rgba="0 0 0.8 1"/>\n')
    f.write('			</material>\n')
    f.write('			<geometry>\n')
    f.write('				<origin xyz="0 0 0" rpy="0 0 0"/>\n')
    f.write('				<cylinder length="0.2" radius="0.2"/>\n')   
    f.write('			</geometry>\n')
    f.write('		</visual>\n')
    f.write('	</link>\n')
    
    f.write(f'	<joint name="poloczenie-{poczatek}-koncowka" type="fixed">\n')
    f.write(f'		<origin xyz="0 0 0"/>\n')
    f.write(f'		<parent link="{poczatek}"/>\n')
    f.write(f'		<child link="koncowka" />\n')
    f.write(f'	</joint>\n')
    
    f.write('</robot>')
    
    	
    print(DHtab)


if __name__ == '__main__':
    main()

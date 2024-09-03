
import math

from geometry_msgs.msg import Point

import matplotlib.pyplot as plt
import networkx as nx



WORLD_NAME = "iot_project_world"

NUMBER_OF_BALLOONS = 7
NUMBER_OF_SENSORS = 1



#-----------------------------------------------------------------------------------------------
# Launch file for the IoT Project. Launches all the nodes required to start the final solution
#-----------------------------------------------------------------------------------------------

graph=nx.Graph()
points = {}
edges = []
def polar_to_euclidian(module,phase,centre):
    # Converte le coordinate polari in cartesiane
    x = module * math.cos(phase)
    y = module * math.sin(phase)
    ret=Point()
    ret.x=centre.x+x
    ret.y=centre.y+y
    return ret
def point_to_tuple(point):
    return (point.x,point.y)
def object_to_point(object):
    ret=Point()
    ret.x=object[0]
    ret.y=object[1]
    return ret
def create_node(integer,point):
    points.update({integer:point})
def create_link(point1,point2):
    edges.append((point1,point2))
def main():
    

    
    balloon_spawn_positions=[]

    #CALCOLA SPAWNING POSITIONS DEI BALLOON
    if NUMBER_OF_BALLOONS<4:
        for i in range(NUMBER_OF_BALLOONS):
            punto=tuple()
            if i==0:
                punto=(0.0, 0.0)
            elif i==1:
                punto = (+32.0, 0.0)
            elif i==2:
                punto = (+16.0,-27.71)
            balloon_spawn_positions.append(object_to_point(punto))

            
    else:
        for i in range(NUMBER_OF_BALLOONS):
            punto=tuple()
            if i%3==0:
                punto=((i/3)*32.0, 0.0)
            elif i%3==1:
                punto=((((i-1)/3)*32.0)+16.0, 27.71)
            else:
                punto=((((i-2)/3)*32.0)+16.0, -27.71)
            balloon_spawn_positions.append(object_to_point(punto))

    for index in range(NUMBER_OF_BALLOONS):
        b_position=balloon_spawn_positions[index]
        #b_position.z=0.701
        create_node(index*7,point_to_tuple(b_position))

        #CREA GRAFO ESAGONO
        for i in range(6):
            new_point=polar_to_euclidian(12,(i*math.pi/3),b_position)
            create_node(index*7+(i+1),point_to_tuple(new_point))
            create_link(index*7,index*7+(i+1))

            if i != 0: 
                create_link(index*7+(i+1),index*7+i)
            if i == 5: 
                create_link(index*7+(i+1),index*7+1)

            
        #CONNETTI COMPONENTI ESAGONALI
        if NUMBER_OF_BALLOONS<4:
            if index==0:pass
            elif index==1:
                #b2_position=balloon_spawn_positions[index-1]
                #b2_position.z=0.701
                #point2=polar_to_euclidian(12,0,b2_position)
                #point1=polar_to_euclidian(12,math.pi,b_position)
                index2=index-1
                create_link(index*7+4,index2*7+1)

            elif index==2:
                #b2_position=balloon_spawn_positions[index-2]
                #b2_position.z=0.701
                #point2=polar_to_euclidian(12,5*math.pi/3,b2_position)
                #point1=polar_to_euclidian(12,2*math.pi/3,b_position)
                #create_link(point1,point2)
                index2=index-2
                create_link(index*7+3,index2*7+6)

                #b2_position=balloon_spawn_positions[index-1]
                #b2_position.z=0.701
                #point2=polar_to_euclidian(12,4*math.pi/3,b2_position)
                #point1=polar_to_euclidian(12,1*math.pi/3,b_position)
                #create_link(point1,point2)
                index2=index-1
                create_link(index*7+5,index2*7+2)
        else:
            if index%3==0:
                if index-3>=0: 
                    #b2_position=balloon_spawn_positions[index-3]
                    #b2_position.z=0.701
                    #point2=polar_to_euclidian(12,0,b2_position)
                    #point1=polar_to_euclidian(12,math.pi,b_position)
                    index2=index-3
                    create_link(index*7+4,index2*7+1)
                if index-2>=0: 
                    #b2_position=balloon_spawn_positions[index-2]
                    #b2_position.z=0.701
                    #point2=polar_to_euclidian(12,5*math.pi/3,b2_position)
                    #point1=polar_to_euclidian(12,2*math.pi/3,b_position)
                    index2=index-2
                    create_link(index*7+3,index2*7+6)
                if index-1>=0: 
                    #b2_position=balloon_spawn_positions[index-1]
                    #b2_position.z=0.701
                    #point2=polar_to_euclidian(12,1*math.pi/3,b2_position)
                    #point1=polar_to_euclidian(12,4*math.pi/3,b_position)
                    index2=index-1
                    create_link(index*7+5,index2*7+2)
            elif index%3==1:
                if index-3>=0: 
                    #b2_position=balloon_spawn_positions[index-3]
                    #b2_position.z=0.701
                    #point2=polar_to_euclidian(12,0,b2_position)
                    #point1=polar_to_euclidian(12,math.pi,b_position)
                    index2=index-3
                    create_link(index*7+4,index2*7+1)
                if index-1>=0: 
                    #b2_position=balloon_spawn_positions[index-1]
                    #b2_position.z=0.701
                    #point2=polar_to_euclidian(12,1*math.pi/3,b2_position)
                    #point1=polar_to_euclidian(12,4*math.pi/3,b_position)
                    index2=index-1
                    create_link(index*7+5,index2*7+2)

            else:
                if index-3>=0: 
                    #b2_position=balloon_spawn_positions[index-3]
                    #b2_position.z=0.701
                    #point2=polar_to_euclidian(12,0,b2_position)
                    #point1=polar_to_euclidian(12,math.pi,b_position)
                    index2=index-3
                    create_link(index*7+4,index2*7+1)
                if index-2>=0: 
                    #b2_position=balloon_spawn_positions[index-2]
                    #b2_position.z=0.701
                    #point2=polar_to_euclidian(12,5*math.pi/3,b2_position)
                    #point1=polar_to_euclidian(12,2*math.pi/3,b_position)
                    index2=index-2
                    create_link(index*7+3,index2*7+6)
    # Creazione del grafo
    G = nx.Graph()

    # Aggiungi vertici (con le loro coordinate nel piano cartesiano)
    #points = {1: (0, 0), 2: (1, 1), 3: (2, 0), 4: (3, 1)}
    for point, coord in points.items():
        G.add_node(point, pos=coord)

    # Aggiungi lati (come tuple dei punti connessi)
    G.add_edges_from(edges)

    # Ottieni le posizioni dei nodi per la visualizzazione
    pos = nx.get_node_attributes(G, 'pos')

    # Disegna i nodi (punti)
    nx.draw_networkx_nodes(G, pos, node_size=300)

    # Disegna i lati (segmenti)
    nx.draw_networkx_edges(G, pos)

    # Aggiungi etichette ai nodi
    nx.draw_networkx_labels(G, pos)

    # Imposta limiti sugli assi
    plt.xlim(-13, 160)
    plt.ylim(-50,50)

    # Mostra il grafico
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()


    return

main()
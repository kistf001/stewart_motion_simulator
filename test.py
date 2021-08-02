import math
import time

def r2a(a):

    return math.pi*(a/180)

def rt_mat(fsaf,fsafd,pppp):

    in_x,    in_y,    in_z    = fsaf[0],fsaf[1],fsaf[2]
    buff_x,  buff_y,  buff_z  = 0,0,0
    theta_x, theta_y, theta_z = r2a(fsafd[0]), r2a(fsafd[1]), r2a(fsafd[2])

    buff_x =  in_x
    buff_y = (in_y*math.cos(theta_x)) - (in_z*math.sin(theta_x))
    buff_z = (in_y*math.sin(theta_x)) + (in_z*math.cos(theta_x))

    buff_x_1 =  (buff_x*math.cos(theta_y)) + (buff_z*math.sin(theta_y))
    buff_y_1 =   buff_y
    buff_z_1 = -(buff_x*math.sin(theta_y)) + (buff_z*math.cos(theta_y))

    buff_x_2 = (buff_x_1*math.cos(theta_z)) - (buff_y_1*math.sin(theta_z))
    buff_y_2 = (buff_x_1*math.sin(theta_z)) + (buff_y_1*math.cos(theta_z))
    buff_z_2 =  buff_z_1

    return  [buff_x_2+pppp[0],  buff_y_2+pppp[1],  buff_z_2+pppp[2]]

def euler_distance(base,platform):

    a = platform[0] - base[0]
    b = platform[1] - base[1]
    c = platform[2] - base[2]

    return (  math.sqrt( (a*a) + (b*b) + (c*c) )  )

def tube_lengther(base_point,plate_point,rotation,transformation,tube):

    aaaaa = []

    for aa in range(0,6):
        aaaaa.append(
            euler_distance( 
                base_point[aa], 
                rt_mat(plate_point[aa],rotation,transformation) 
            ) * tube
        )
    
    return aaaaa

tube_length = 600
base_point = []
plate_point = []

# base plate
base_point.append(  rt_mat([1,0,0],[0,0, 30],[0,0,0])  )
base_point.append(  rt_mat([1,0,0],[0,0, 90],[0,0,0])  )
base_point.append(  rt_mat([1,0,0],[0,0,150],[0,0,0])  )
base_point.append(  rt_mat([1,0,0],[0,0,210],[0,0,0])  )
base_point.append(  rt_mat([1,0,0],[0,0,270],[0,0,0])  )
base_point.append(  rt_mat([1,0,0],[0,0,330],[0,0,0])  )
print("base plate point : ",base_point)

# plate plate
plate_point.append(  rt_mat([1,-0.1,1],[0,0, 60],[0,0,0])  )
plate_point.append(  rt_mat([1, 0.1,1],[0,0, 60],[0,0,0])  )
plate_point.append(  rt_mat([1,-0.1,1],[0,0,180],[0,0,0])  )
plate_point.append(  rt_mat([1, 0.1,1],[0,0,180],[0,0,0])  )
plate_point.append(  rt_mat([1,-0.1,1],[0,0,300],[0,0,0])  )
plate_point.append(  rt_mat([1, 0.1,1],[0,0,300],[0,0,0])  )
#print("1 point : ",plate_point)

#print(    tube_lengther(base_point,plate_point,[0,0,0],[0.3,0,0],tube_length)    )
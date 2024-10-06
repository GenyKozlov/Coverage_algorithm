"""

LIDAR to 2D grid map example

author: Erno Horvath, Csaba Hajdu based on Atsushi Sakai's scripts (@Atsushi_twi)

"""

import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from matplotlib.patches import Polygon



class GridMap:
    # Задание значения по умолчанию для начальной точки
    def __init__(self, polygon_vertices, point_inside_polygon=[0,0]):
        self.map_center = np.array([0.0, 0.0])
        self.map_width_m = 5.0
        self.map_length_m = 5.0
        self.map_resolution_m = 0.01 # [m]
        self.flight_area_vertices = polygon_vertices
        
        self.create_borders_grid_map(point_inside_polygon)

    def create_borders_grid_map(self, polygon_center):
        WIDTH = int((self.map_width_m) / self.map_resolution_m) # количество строк в gmap
        LENGTH = int((self.map_length_m) / self.map_resolution_m) # количество столбцов в gmap
        gmap = np.ones([WIDTH, LENGTH])
        points = self.meters2grid(self.flight_area_vertices).tolist() # превращает массив в список, чтобы можно было добавить элементы
        points.append(points[0])
        for i in range(len(points)-1):
            line = bresenham(points[i], points[i+1])
            for l in line: # для каждой точки по алгоритму Брезенхэма
                gmap[l[1]][l[0]] = 0 # ноль на месте точки в матрице gmap
        polygon_center_grid = np.array( self.meters2grid(polygon_center), dtype=int ) # передача в качестве аргумента начального положения БПЛА (начальной точки)
        gmap = flood_fill(polygon_center_grid, gmap) # передача аргументами центра сетчатой карты (это сам БПЛА) и матрицы
        self.gmap = gmap

    def add_obstacles_to_grid_map(self, obstacles): # получает координаты вершин препятствия
        """ Obstacles dicretized map """
        # rectangular obstacles, берется левый нижний и правый верхний вершины
        for obstacle in obstacles:
            x1 = self.meters2grid(obstacle[0][1]); x2 = self.meters2grid(obstacle[2][1]) # перевод метрических координат в сеточные координаты
            y1 = self.meters2grid(obstacle[0][0]); y2 = self.meters2grid(obstacle[2][0])
            if x1 > x2: tmp = x2; x2 = x1; x1 = tmp
            if y1 > y2: tmp = y2; y2 = y1; y1 = tmp
            self.gmap[x1:x2, y1:y2] = 1 # единицы в матрице на месте препятствия (по всему прямоугольнику, не только по границе)
        return self.gmap

    def meters2grid(self, pose_m,): # выводит координаты точек в сетчатом пространстве, где начало координат лежит в левом нижнем углу
        nrows = int(self.map_width_m / self.map_resolution_m)
        ncols = int(self.map_length_m / self.map_resolution_m)
        if np.isscalar(pose_m):
            pose_on_grid = int( pose_m/self.map_resolution_m + ncols/2 )
        else:
            pose_on_grid = np.array( np.array(pose_m)/self.map_resolution_m +\
                                     np.array([ncols/2, nrows/2]) -\
                                     self.map_center/self.map_resolution_m, dtype=int ) # точки целочисленные
        return pose_on_grid
    
    def grid2meters(self, pose_grid):
        # [100, 100] -> [0, 0](m)
        # [100+100, 100] -> [1, 0](m)
        # [100, 100-100] -> [0,-1](m)
        nrows = int(self.map_width_m / self.map_resolution_m)
        ncols = int(self.map_length_m / self.map_resolution_m)
        if np.isscalar(pose_grid):
            pose_meters = (pose_grid - ncols/2) * self.map_resolution_m
        else:
            pose_meters = ( np.array(pose_grid) - np.array([ncols/2, nrows/2]) ) *\
                            self.map_resolution_m - self.map_center
        return pose_meters

    def draw_map(self, obstacles):
        ax = plt.gca()
        w = self.map_width_m; l = self.map_length_m
        ax.set_xlim([-w/2, w/2])
        ax.set_ylim([-l/2, l/2])
        boundaries = self.flight_area_vertices
        ax.add_patch( Polygon(boundaries, linewidth=2, edgecolor='k',facecolor='none') )
        for k in range(len(obstacles)):
            ax.add_patch( Polygon(obstacles[k]+self.map_center, color='k', zorder=10) )



def bresenham(start, end):
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end (original from roguebasin.com)
    >>> points1 = bresenham((4, 4), (6, 10))
    >>> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx) # determine how steep the line is
    if is_steep: # rotate line (симметрия относительно прямой +45 градусов)
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    swapped = False # swap start and end points if necessary and store swap state
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1 # recalculate differentials
    dy = y2 - y1 # recalculate differentials
    error = int(dx / 2.0) # calculate error - округление к меньшему целому числу
    ystep = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    # перебор всех х от х1 до х2 
    for x in range(x1, x2 + 1):
        # возвращает координаты назад, если производили симметрию относительно прямой +45 градусов
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)   
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
    if swapped: # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points # возвращает массив точек по алгоритму Брезенхэма



def flood_fill(cpoint, pmap): # передача аргументами центра сетчатой карты (это сам БПЛА) и матрицы
    """
    cpoint: starting point (x,y) of fill
    pmap: occupancy map generated from Bresenham ray-tracing
    """
    # Fill empty areas with queue method - заполнение 
    sx, sy = pmap.shape # размерность матрицы: кол-во строк и столбцов
    fringe = deque() # создание контейнера
    fringe.appendleft(cpoint) # добавление центральной точки слева контейнера (в его начало)
    while fringe:
        n = fringe.pop() # удаляется и записывается в n правый элемент контейнера (последний)
        nx, ny = n
        # West
        if nx > 0:
            if pmap[nx - 1, ny] == 1.0:
                pmap[nx - 1, ny] = 0.0
                fringe.appendleft((nx - 1, ny))
        # East
        if nx < sx - 1:
            if pmap[nx + 1, ny] == 1.0:
                pmap[nx + 1, ny] = 0.0
                fringe.appendleft((nx + 1, ny))
        # North
        if ny > 0:
            if pmap[nx, ny - 1] == 1.0:
                pmap[nx, ny - 1] = 0.0
                fringe.appendleft((nx, ny - 1))
        # South
        if ny < sy - 1:
            if pmap[nx, ny + 1] == 1.0:
                pmap[nx, ny + 1] = 0.0
                fringe.appendleft((nx, ny + 1))
    return pmap # нули в матрице будут на месте координат точек полигона, за пределами - единицы

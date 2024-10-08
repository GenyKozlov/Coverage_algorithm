"""
Grid based sweep planner

author: Atsushi Sakai
"""

import math
import os
import sys
from enum import IntEnum

import matplotlib.pyplot as plt
import numpy as np

from grid_map_lib import GridMap


class SweepSearcher:
    class SweepDirection(IntEnum):
        UP = 1
        DOWN = -1

    class MovingDirection(IntEnum):
        RIGHT = 1
        LEFT = -1

    def __init__(self, mdirection, sdirection, xinds_goaly, goaly): # moving direction = Right, sweeping_direction = Up
        self.moving_direction = mdirection
        self.sweep_direction = sdirection
        self.turing_window = []
        self.update_turning_window()
        self.xinds_goaly = xinds_goaly
        self.goaly = goaly

    def move_target_grid(self, cxind, cyind, gmap):
        nxind = self.moving_direction + cxind
        nyind = cyind

        # found safe grid
        if not gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):
            return nxind, nyind
        else:  # occupided
            ncxind, ncyind = self.find_safe_turning_grid(cxind, cyind, gmap)
            if (ncxind is None) and (ncyind is None):
                # moving backward
                ncxind = -self.moving_direction + cxind
                ncyind = cyind
                if gmap.check_occupied_from_xy_index(ncxind, ncyind):
                    # moved backward, but the grid is occupied by obstacle
                    return None, None
            else:
                # keep moving until end
                while not gmap.check_occupied_from_xy_index(ncxind + self.moving_direction, ncyind, occupied_val=0.5):
                    ncxind += self.moving_direction
                self.swap_moving_direction()
            return ncxind, ncyind

    def find_safe_turning_grid(self, cxind, cyind, gmap):

        for (dxind, dyind) in self.turing_window:

            nxind = dxind + cxind
            nyind = dyind + cyind

            # found safe grid
            if not gmap.check_occupied_from_xy_index(nxind, nyind, occupied_val=0.5):
                return nxind, nyind

        return None, None

    def is_search_done(self, gmap):
        for ix in self.xinds_goaly:
            if not gmap.check_occupied_from_xy_index(ix, self.goaly, occupied_val=0.5):
                return False

        # all lower grid is occupied
        return True

    def update_turning_window(self): # moving direction = Right, sweeping_direction = Up
        self.turing_window = [
            (self.moving_direction, 0.0),
            (self.moving_direction, self.sweep_direction),
            (0, self.sweep_direction),
            (-self.moving_direction, self.sweep_direction),
        ]

    def swap_moving_direction(self):
        self.moving_direction *= -1
        self.update_turning_window()

    def search_start_grid(self, grid_map):
        xinds = []
        y_ind = 0
        if self.sweep_direction == self.SweepDirection.DOWN:
            xinds, y_ind = search_free_grid_index_at_edge_y(grid_map, from_upper=True)
        elif self.sweep_direction == self.SweepDirection.UP:
            xinds, y_ind = search_free_grid_index_at_edge_y(grid_map, from_upper=False)

        if self.moving_direction == self.MovingDirection.RIGHT:
            return min(xinds), y_ind
        elif self.moving_direction == self.MovingDirection.LEFT:
            return max(xinds), y_ind

        raise ValueError("self.moving direction is invalid ")


def find_sweep_direction_and_start_posi(ox, oy):
    # find sweep_direction
    max_dist = 0.0
    vec = [0.0, 0.0]
    sweep_start_pos = [0.0, 0.0]
    for i in range(len(ox) - 1):
        dx = ox[i + 1] - ox[i] # смещение по координате х между соседними точками
        dy = oy[i + 1] - oy[i] # смещение по координате у между соседними точками
        d = np.sqrt(dx ** 2 + dy ** 2) # гипотенуза

        if d > max_dist: # поиск максимального расстояния между точками
            max_dist = d # сохранение расстояния
            vec = [dx, dy] # сохранение смещения
            sweep_start_pos = [ox[i], oy[i]] # сохранение координат точки, которая является началом в данной паре

    return vec, sweep_start_pos # возвращает смещение и начальную точку из пары


def convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_posi):
    tx = [ix - sweep_start_posi[0] for ix in ox] # из всех координат х в ох вычитают координату х точки начала развертки
    ty = [iy - sweep_start_posi[1] for iy in oy] # из всех координат у в оу вычитают координату у точки начала развертки

    th = math.atan2(sweep_vec[1], sweep_vec[0]) # угол между прямой, начало которой является началом развертки, и осью х в радианах (от -pi до pi)

    c = np.cos(-th) # расчет косинуса для угла в радианах
    s = np.sin(-th) # расчет синуса для угла в радианах

    rx = [ix * c - iy * s for (ix, iy) in zip(tx, ty)] # zip временно рассматривает два списка как один, состоящий из списков координат всех точек
    ry = [ix * s + iy * c for (ix, iy) in zip(tx, ty)]

    # данная функция создает новую систему координат, начало которой расположено в начале развертки, ось х направлена вдоль прямой с наибольшим расстоянием между
    # двумя точками, начало которой в начале развертки (d), rx и ry - координаты всех точек полигона в новой системе координат
    # цепь замкнута

    return rx, ry


def convert_global_coordinate(x, y, sweep_vec, sweep_start_posi):
    th = math.atan2(sweep_vec[1], sweep_vec[0])
    c = np.cos(th)
    s = np.sin(th)

    tx = [ix * c - iy * s for (ix, iy) in zip(x, y)]
    ty = [ix * s + iy * c for (ix, iy) in zip(x, y)]

    rx = [ix + sweep_start_posi[0] for ix in tx]
    ry = [iy + sweep_start_posi[1] for iy in ty]

    return rx, ry


def search_free_grid_index_at_edge_y(grid_map, from_upper=False):
    yind = None
    xinds = []

    if from_upper:
        xrange = range(int(grid_map.height))[::-1] # если направление развертки вверх, шаг среза = -1, элементы выбираются справа налево
        yrange = range(int(grid_map.width))[::-1]
    else:
        xrange = range(int(grid_map.height))
        yrange = range(int(grid_map.width))

    for iy in xrange:
        for ix in yrange:
            if not grid_map.check_occupied_from_xy_index(ix, iy):
                yind = iy
                xinds.append(ix)
        if yind:
            break

    return xinds, yind


def setup_grid_map(ox, oy, reso, sweep_direction, offset_grid=10):
    # передаются координаты вершин полигона в новой системе координат, разрешение и направление

    # карта охватывает весь полигон, центр карты
    width = math.ceil((max(ox) - min(ox)) / reso) + offset_grid # ceil округляет число в большую сторону: 5,3 - 6; -4,8 - -4
    height = math.ceil((max(oy) - min(oy)) / reso) + offset_grid
    center_x = np.mean(ox) # mean - рассчитывает среднее арифметическое значение элементов массива
    center_y = np.mean(oy)

    grid_map = GridMap(width, height, reso, center_x, center_y) # принимает длину (по х), ширину (по у), разрешение и координаты центра

    grid_map.set_value_from_polygon(ox, oy, 1.0, inside=False) # передаются координаты вершин полигона в новой системе координат

    grid_map.expand_grid()

    xinds_goaly = []
    goaly = 0
    if sweep_direction == SweepSearcher.SweepDirection.UP:
        xinds_goaly, goaly = search_free_grid_index_at_edge_y(grid_map, from_upper=True)
    elif sweep_direction == SweepSearcher.SweepDirection.DOWN:
        xinds_goaly, goaly = search_free_grid_index_at_edge_y(grid_map, from_upper=False)
    return grid_map, xinds_goaly, goaly


def sweep_path_search(sweep_searcher, gmap, grid_search_animation=False):
    # search start grid
    cxind, cyind = sweep_searcher.search_start_grid(gmap)
    if not gmap.set_value_from_xy_index(cxind, cyind, 0.5):
        print("Cannot find start grid")
        return [], []

    x, y = gmap.calc_grid_central_xy_position_from_xy_index(cxind, cyind)
    px, py = [x], [y]

    print(grid_search_animation)
    if grid_search_animation:
        fig, ax = plt.subplots()

    while True:
        cxind, cyind = sweep_searcher.move_target_grid(cxind, cyind, gmap)

        if sweep_searcher.is_search_done(gmap) or (cxind is None or cyind is None):
            print("Done")
            break

        x, y = gmap.calc_grid_central_xy_position_from_xy_index(
            cxind, cyind)

        px.append(x)
        py.append(y)

        gmap.set_value_from_xy_index(cxind, cyind, 0.5)

        if grid_search_animation:
            gmap.plot_grid_map(ax=ax)
            plt.pause(1.0)

    return px, py # возвращает координаты целевых точек в сеточной карте


def planning(ox, oy, reso,
             moving_direction=SweepSearcher.MovingDirection.RIGHT,
             sweeping_direction=SweepSearcher.SweepDirection.UP,
             ):
    sweep_vec, sweep_start_posi = find_sweep_direction_and_start_posi(ox, oy) # принимает срезы по координатам х и у вершин

    rox, roy = convert_grid_coordinate(ox, oy, sweep_vec, sweep_start_posi) # принимает срезы по координатам, смещение и начальную точку из пары 
    # (для которой наибольшее расстояние между точками)
    # rox и roy - координаты вершин полигона в новой системе координат, цепь замкнута
    
    gmap, xinds_goaly, goaly = setup_grid_map(rox, roy, reso, sweeping_direction)

    sweep_searcher = SweepSearcher(moving_direction, sweeping_direction, xinds_goaly, goaly) 

    px, py = sweep_path_search(sweep_searcher, gmap)
    # px и pу - это список координат х и у целевых точек в сеточной карте

    rx, ry = convert_global_coordinate(px, py, sweep_vec, sweep_start_posi)
    # rx и ry - это список координат х и у целевых точек на глоабльной карте

    print("Path length:", len(rx))

    return rx, ry
import time
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

def polygon_contains_point(point, polygon_vertices):
	point = Point(point[0], point[1])
	polygon = Polygon(polygon_vertices)
	return polygon.contains(point)

def define_polygon(num_pts=4):
	def tellme(s):
	    print(s)
		# вывести заголовок у рисунка - "s"
	    plt.title(s, fontsize=16)
		# перерисовать рисунок, т.к. его изменили
	    plt.draw()
	
	# текущие экземпляры осей
	ax = plt.gca()
	# изменение параметров объекта
	plt.setp(ax, autoscale_on=0)
	# пределы просмотра по осям
	ax.set_xlim([-2.5, 2.5])
	ax.set_ylim([-2.5, 2.5])

	# вывести в консоль и на график предложение с количеством строк, равным "num_pts"
	tellme('You will define a flight area with %d points.\nClick to begin.'%num_pts)

	# вызов взаимодействия с фигурой: True - если нажата клавиша, False - если кнопка мыши
	plt.waitforbuttonpress()
	
	while True:
	    pts = []
	    while len(pts) < num_pts:
	        tellme('Select %d corners with mouse'%num_pts)
			# преобразование входных данных в массив: ginput возвращает координаты каждого нажатия по рисунку [[x1,y2], [x2,y2],...]
	        pts = np.asarray(plt.ginput(num_pts, timeout=-1))
			# проверка на количество выбранных точек
	        if len(pts) < num_pts:
	            tellme('Too few points, starting over')
	            time.sleep(1)

		# заливка полигона
	    ph = plt.fill(pts[:, 0], pts[:, 1], 'r', lw=2)

	    tellme('Happy? Press any key for YES, mouse click for NO')

		# если нажата клавиша, прервать цикл
	    if plt.waitforbuttonpress():
	        break

	    # Удаление заливки полигона
	    for p in ph:
	        p.remove()
	
	return pts
import math

def mmin(mas):
	k = [100000, 'a']
	for x in mas:
		if x[0] < k[0]:
			k = x
	return k

def mmax(mas):
	k = [-100000, 'a']
	for x in mas:
		if x[0] > k[0]:
			k = x
	return k

def distance(pointStart, pointEnd):
	return getModule(pointStart, pointEnd)

def getModule(pointStart, pointEnd):
	return math.sqrt((pointStart.lon - pointEnd.lon) ** 2 + (pointStart.lat - pointEnd.lat) ** 2 )

class Point(object):
	"""docstring for Point"""

	lat = None
	lon = None
	alt = 0
	time = None

	def __init__(self, lat, lon, time = None, alt = 0):
		self.lat = lat
		self.lon = lon
		self.time = time

	def __sub__(self, other):
		return Point(self.lat - other.lat, self.lon - other.lon)

	@staticmethod
	def toPoint(res):
		mas = res.split(' ')
		mas[0] = mas[0][6:]
		mas[1] = mas[1][:-1]
		return Point(float(mas[1]),float(mas[0]))

	def AsWKT(self):
		"""Present point on WKT style POINT(lon, lat)"""
		return 'POINT('+str(self.lon)+' '+str(self.lat)+')'

	def __add__(self, other):
		return Point(self.lat + other.lat, self.lon + other.lon)

	def __str__(self):
		return "["+str(self.lat)+", "+str(self.lon)+"]"

	def __mul__(self, other):
		return Point(self.lat * other, self.lon * other)

	def getModule(self):
		return math.sqrt(self.lon ** 2 + self.lat ** 2)


class Edge(object):
	"""docstring for Edge"""
	id = None
	src = None
	trg = None
	srcid = None
	trgid = None
	x = 0
	y = 0
	speed = None

	def __init__(self, src, trg, id = None, speed = 40):
		if type(src) == Point:
			self.src = src
		else:
			self.src = src.point
			self.srcid = src.id

		if type(src) == Point:
			self.trg = trg
		else:
			self.trg = trg.point
			self.trgid = trg.id
		self.id = id
		self.speed = speed

		self.x = self.trg.lat - self.src.lat
		self.y = self.trg.lon - self.src.lon
	

	def getModule(self):
		"""Calculate distance between two points
			return distance in meters"""
		#if self.src.converted == False or self.trg.converted == False:
		#	self.convertLine()

		#return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)
		return math.sqrt(self.x ** 2 + self.y ** 2)

	def reverse(self):
		return Edge(Node(self.trgid, self.trg.lat, self.trg.lon),Node(self.srcid, self.src.lat, self.src.lon), self.id, self.speed)

	def __str__(self):
		return str(self.src)+","+str(self.trg)

	def __mul__(self, other):
		return Point(self.x * other, self.y * other)

def scalar(edgeFirst, edgeSecond):
	return (edgeFirst.x*edgeSecond.x + edgeFirst.y*edgeSecond.y)


def deltaAngle(edgeFirst, edgeSecond):
	return scalar(edgeFirst, edgeSecond)/(edgeFirst.getModule()*edgeSecond.getModule())

class Node():
	id = 0
	point = None
	visited = False

	def __init__(self, id, lat, lon):
		self.id = id
		self.point = Point(lat,lon)

	def __str__(self):
		return "id = " +str(self.id)+", latlon =" + str(self.point)


def getPointAsWKT(s):
	s = s.split(' ')
	lon = s[0][6:]
	lat = s[1][:-1]
	return Point(lat, lon)

def dist(point, edge):
	x1 = edge.src.lat
	y1 = edge.src.lon
	x2 = edge.trg.lat
	y2 = edge.trg.lon
	x3 = point.lat
	y3 = point.lon

	px = x2-x1
	py = y2-y1

	px2 = x1-x2
	py2 = y1-y2

	length = px*px + py*py

	try:
		u =  ((x3 - x1) * px + (y3 - y1) * py)/length
	except ZeroDivisionError:
		print('bad Edge: '+str(edge.id))
		return 1000, 1

	if u > 1:
		return min(math.sqrt((x3-x2) ** 2 + (y3-y2) ** 2),math.sqrt((x3-x1) ** 2 + (y3-y1) ** 2)), u
	elif u < 0:
		return min(math.sqrt((x3-x2) ** 2 + (y3-y2) ** 2),math.sqrt((x3-x1) ** 2 + (y3-y1) ** 2)), u

	x = x1 + u * px
	y = y1 + u * py

	dx = x - x3
	dy = y - y3
	dist = math.sqrt(dx ** 2 + dy ** 2)

	return dist, u

def probDistWithAngle(ordinal, candidateEdge, lastOrdinal):
	""" Calculate observe propability
		Attributes: 
		ordinal -- GPS track point
		candidateEdge -- Candidate edge
	"""

	sigmaO = 20
	sigmaA = 0.71

	h,u = dist(ordinal, candidateEdge)
	if u < 0 or u > 1:
		distanc = distance(candidateEdge.trg, ordinal)*111.11111*1000
	else:
		distanc = h*111.11111*1000
	obs = (1/2*(1 + math.erfc(distanc/math.sqrt(2*sigmaO**2))))**2
	ang = 1/2*(1 + math.erfc(1 - deltaAngle(Edge(lastOrdinal,ordinal), candidateEdge) ** 2)/math.sqrt(2* sigmaA ** 2))
	
	return obs*ang
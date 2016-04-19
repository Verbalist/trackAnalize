import postgresql
from tools2 import Point
from tools2 import Node
from tools2 import Edge

vecCon = postgresql.open(user = 'postgres', host = 'localhost', port = 5432, password='1111', database='mapvector2')
trackCon = postgresql.open(user = 'postgres', host = 'localhost', port = 5432, password='1111', database='tracksDB2')

def getNeighbors(point,R=1):
	"""Selection nearest nodes for point wits coordinates(lat,lon) in radius R"""

	queryString = "SELECT id, ST_Y(coord::geometry) AS lat, ST_X(coord::geometry) AS lon FROM nodes WHERE ST_DWithin(coord, ST_GeographyFromText($1), $2) ORDER BY ST_Distance(coord, ST_GeographyFromText($1));"
	pr = vecCon.prepare(queryString)
	
	return [Node(x[0], x[1], x[2]) for x in pr(point.AsWKT(), R*1000)]

def getEdges(point,R=1):
	"""Selection nearest edges for point wits coordinates(lat,lon) in radius R"""

	queryString = "SELECT id, srcid, (select ST_AsText(coord) FROM nodes WHERE id=srcid) AS src, trgid, (select ST_AsText(coord) FROM nodes WHERE id=trgid) AS trg, speed FROM edges WHERE (srcid IN (SELECT id FROM nodes WHERE ST_DWithin(coord, ST_GeographyFromText($1), $2)) and trgid in (SELECT id FROM nodes WHERE ST_DWithin(coord, ST_GeographyFromText($1), $2)));"
	pr = vecCon.prepare(queryString)
	ret = []
	for row in pr(point.AsWKT(),R*1000):
		latlon1 = row[2].split(" ")
		latlon2 = row[4].split(" ")

		src = Node(row[1], float(latlon1[1][:-1]), float(latlon1[0][6:]))
		trg = Node(row[3], float(latlon2[1][:-1]), float(latlon2[0][6:]))
		ret.append(Edge(src, trg, row[0],row[5]))
	return ret

def getNearest(point):
	"""Selection nearest node for point wits coordinates(lat,lon)"""

	queryString = "SELECT id, ST_Y(coord::geometry) AS lat, ST_X(coord::geometry) AS lon FROM nodes WHERE ST_DWithin(coord, ST_GeographyFromText($1), $2) ORDER BY ST_Distance(coord, ST_GeographyFromText($1)) LIMIT 1;"
	pr = vecCon.prepare(queryString)
	R = 100
	ret = pr(point.AsWKT(),R)
	while (ret == []):
		R *= 10
		ret = pr(point.AsWKT(),R)
	return Node(ret[0][0],ret[0][1],ret[0][2])

def getTracks(transpid, time):
	"""Selection tracks if track_time < time"""

	queryString = "SELECT lat/1000000::float as lat, lon/1000000::float as lon, time, alt, speed, direction FROM points WHERE (time > $2 and transpid = $1) ORDER BY time;"
	pr = trackCon.prepare(queryString)
	ret = []
	for x in pr(transpid, time):
		ret.append([Point(x[0], x[1], x[2], x[3]), x[4], x[5]])
	return ret

def getTracks2(transpid):
	"""Selection tracks if track_transp_id = transpid"""
	odCon = postgresql.open(user = 'postgres', host = 'localhost', port = 5432, password='1111', database='odessapoint')
	queryString = "SELECT lat, lon, time, alt, speed, direction FROM tracks2 WHERE (transpid=$1 and speed!=0) ORDER BY time;"
	pr = odCon.prepare(queryString)
	ret = []
	for x in pr(transpid):
		ret.append([Point(x[0], x[1], x[2], x[3]), x[4], x[5]])
	return ret

def getTracks3(transpid, time):
	"""Selection tracks if track_transp_id = transpid"""
	odCon = postgresql.open(user = 'postgres', host = 'localhost', port = 5432, password='1111', database='odessapoint')
	queryString = "SELECT lat, lon, time, alt, speed, direction FROM tracks2 WHERE (transpid=$1 and time>$2) ORDER BY time;"
	pr = odCon.prepare(queryString)
	ret = []
	for x in pr(transpid,time):
		ret.append([Point(x[0], x[1], x[2], x[3]), x[4], x[5]])
	return ret

def getRoute(id):
	queryString = "SELECT id, srcid, (select ST_AsText(coord) FROM nodes WHERE id=srcid) AS src, trgid, (select ST_AsText(coord) FROM nodes WHERE id=trgid) AS trg , speed FROM edges WHERE id=$1;"
	pr = vecCon.prepare(queryString)
	row = pr(id)[0]
	latlon1 = row[2].split(" ")
	latlon2 = row[4].split(" ")
	src = Node(row[1], float(latlon1[1][:-1]), float(latlon1[0][6:]))
	trg = Node(row[3], float(latlon2[1][:-1]), float(latlon2[0][6:]))
	
	return Edge(src, trg, row[0],row[5])


def setNeighborsD(point,R=1):
	"""Selection nearest nodes for point wits coordinates(lat,lon) in radius R"""

	queryString = "explain SELECT * FROM edges WHERE ((srcid = ANY(SELECT id FROM nodes WHERE ST_DWithin(coord, ST_GeographyFromText($1), $2))) or (trgid = ANY(SELECT id FROM nodes WHERE ST_DWithin(coord, ST_GeographyFromText($1), $2))));"
	pr = vecCon.prepare(queryString)
	#print(pr(point.AsWKT(),R*1000))

def getNext(id):
	queryString	= "SELECT id FROM edges WHERE ((srcid=$1 and forward=True) or (trgid=$1 and backward=True));"
	pr = vecCon.prepare(queryString)
	return [x[0] for x in pr(id)]

def getNextNode(id):
	queryString = "SELECT trgid FROM edges WHERE (srcid=$2 AND forward=True) UNION SELECT srcid FROM edges WHERE (trgid=$2 AND backward=True))"
	pr = vecCon.prepare(queryString)
	return [x for x in pr(id)]

def getNextEdge(id, backward=False):
	"""Return list Edges that start and finish in node with id=id"""
	if backward == False:
		queryString = "SELECT id, srcid, (select ST_AsText(coord) FROM nodes WHERE id=srcid) AS src, trgid, (select ST_AsText(coord) FROM nodes WHERE id=trgid) AS trg , speed FROM edges WHERE id IN (SELECT id FROM edges WHERE (((srcid=$1 and forward=True) or (trgid=$1 and backward=True))) and confirmed=TRUE);"
	else:
		queryString = "SELECT id, srcid, (select ST_AsText(coord) FROM nodes WHERE id=srcid) AS src, trgid, (select ST_AsText(coord) FROM nodes WHERE id=trgid) AS trg , speed FROM edges WHERE id IN (SELECT id FROM edges WHERE (((srcid=$1 and backward=True) or (trgid=$1 and forward=True))) and confirmed=TRUE);"
	pr = vecCon.prepare(queryString)
	ret = []
	for row in pr(id):
		latlon1 = row[2].split(" ")
		latlon2 = row[4].split(" ")

		if id == row[1]:
			src = Node(row[1], float(latlon1[1][:-1]), float(latlon1[0][6:]))
			trg = Node(row[3], float(latlon2[1][:-1]), float(latlon2[0][6:]))
		else:
			src = Node(row[3], float(latlon2[1][:-1]), float(latlon2[0][6:]))
			trg = Node(row[1], float(latlon1[1][:-1]), float(latlon1[0][6:]))
		ret.append(Edge(src, trg, row[0],row[5]))
	return ret

def getNerestInNext(point, nearest):
	queryString = "SELECT id, ST_Y(coord::geometry) AS lat, ST_X(coord::geometry) AS lon, ST_Distance(coord, ST_GeographyFromText($1)) FROM nodes WHERE id IN (SELECT trgid FROM edges WHERE (srcid=$2 AND forward=True) UNION SELECT srcid FROM edges WHERE (trgid=$2 AND backward=True)) ORDER BY ST_Distance(coord, ST_GeographyFromText($1));"
	pr = vecCon.prepare(queryString)
	ret = []
	for x in pr(point.AsWKT(), nearest.id):
		ret.append([Node(x[0],x[1],x[2]),x[3]])
	return ret

def getEdgeDate(srcid, trgid):
	queryString = "SELECT id, speed FROM edges WHERE ((srcid=$1 and trgid=$2) or (srcid=$1 and trgid=$2));"
	pr = vecCon.prepare(queryString)
	#print(pr(srcid, trgid))
	ret = pr(srcid, trgid)[0]
	return ret[0], ret[1]

def setNewRoad(points):
	copy = vecCon.prepare("COPY new_road(transpid, lat, lon, time) FROM stdin;")
	copy.load_rows(points)

def setNewSpeed(speeds):
	copy = vecCon.prepare("COPY speed_on_edge(transpid, time, edge_id, speed) FROM stdin;")
	copy.load_rows(speeds)

def getTransports():
	queryString = "SELECT globalid, last_time_analyze, is_analyze FROM transports;"
	pr = trackCon.prepare(queryString)
	ret = []
	for row in pr():
		if row[2] == True:
			ret.append([row[0],row[1]])
	return ret

def setTime(time, transpid):
	queryString = "UPDATE transports SET last_time_analyze="+str(time)+" where id="+str(transpid)+";"
	trackCon.execute(queryString)

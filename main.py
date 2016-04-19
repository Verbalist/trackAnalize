import db
import time
from tools2 import *
import math
from prob import P
from adaptiveFilter import *


def main(transpid, time, deep = 10, maxDeep = 25, R = 0.3):

	"""const"""
	deep = 10
	maxDeep = 25
	R = 0.3

	pointSet = []
	for x in db.getTracks(transpid, time):
		pointSet.append(x[0])

	db.setTime(pointSet[-1].time, transpid)

	print("len pointSet: " + str(len(pointSet)))
	def segmentation(pointSet):
		pointList = []
		z = 0
		while (True):
			try:
				if (distance(pointSet[z], pointSet[z+1])*111.1111*1000 > 30 and distance(pointSet[z], pointSet[z+1])*111.1111*1000 < 60):
					pointList.append(pointSet[z])
					pointList.append(midPoint(pointSet[z],pointSet[z+1]))
				elif distance(pointSet[z], pointSet[z+1])*111.1111*1000 < 1:
					pass
				else: 
					pointList.append(pointSet[z])
				z += 1
			except IndexError:
				return pointList

	def midPoint(pointFirst, pointSecond):
		mid = (pointSecond-pointFirst)*0.5
		midPoint = pointFirst + mid
		midPoint.time = pointFirst.time + (pointSecond.time - pointFirst.time)/2
		return midPoint

	pointList = segmentation(pointSet)

	# rlat = prohod([x.lat for x in pointList],10000000,5)

	# rlon = prohod([x.lon for x in pointList],10000000,5)

	# lats = [x.lat for x in pointList]
	# lons = [x.lon for x in pointList]
	# for i in range(5,len(pointList)):
	# 	pointList[i].lat = rlat.progn(lats[i-5:i])
	# for i in range(5,len(pointList)):
	# 	pointList[i].lon = rlon.progn(lons[i-5:i])


	pointList=pointList[10:]
	nearestList = []
	pointOnEdge = []


	def search(first = 0, iter = None, backward = False, debug = False, propability = 0.53, maxDeep = 25, R = 0.3):
		"""
			linking track to edges 

			Attributes: 
				first -- point of start algorithm
				iter -- point of end algorithm
				backward -- flag for reverse search
				debug -- flag for output more information
				propability -- if propability linking < propability -> out of algorithm
				nearest -- first edge on route
				maxDeep -- max deep search for back search

			return:
				forward:
					i -- iterator of exit
				backward:
					[ how many step did algorithm,
					  emergency exit ]
			"""

		i = first
		MAX = len(pointList)
		end = len(pointList) - 1
		if iter != None:
			end = iter

		
		def startStep(i, debug, R):
			"""
			prepare step for algorithm for linking track 

			Attributes: 
				i -- global iterator in search
				debug -- flag for output more information

			return:
				i -- iterator of exit
				nearest -- first edge on route
			"""

			startEdges = [[probDistWithAngle(pointList[i],edge, pointList[i-1]),edge] for edge in db.getEdges(pointList[i],R)]
			if startEdges == []:
				Rb = R + 0.5
				while startEdges == []:
					startEdges = [[probDistWithAngle(pointList[i],edge, pointList[i-1]),edge] for edge in db.getEdges(pointList[i],Rb)]
					Rb += 0.5
			candidate = mmax(startEdges)[1]

			nearest = candidate
			nearestList.append(candidate)
			while (i < end or backward) and i < MAX:
				h,u = dist(pointList[i], candidate)
				if u > 1 or u < 0:
					dsrc = distance(pointList[i],candidate.src)
					dtrg = distance(pointList[i],candidate.trg)
					if dsrc < dtrg:
						x = candidate.src
						xid = candidate.srcid
						candidate.src = candidate.trg
						candidate.srcid = candidate.trgid
						candidate.trg = x
						candidate.trgid = xid
						lastOrdinal = candidate.trg
						lastOrdinal.time = pointList[i].time
						return nearest, i
					else:
						lastOrdinal = candidate.trg
						lastOrdinal.time = pointList[i].time
						return nearest, i
				else:
					try:
						x = math.sqrt((pointList[i]-candidate.src).getModule() ** 2 - h ** 2)
						lastOrdinal = candidate*(x/candidate.getModule()) + candidate.src
					except ValueError:
						lastOrdinal = candidate.src
					lastOrdinal.time = pointList[i].time
					pointOnEdge.append([lastOrdinal, candidate.id, i])
					i += 1

		nearest, i = startStep(i, debug, R)
		if pointOnEdge == []:
			return i + 1

		def mainStep(nearest, i, debug, backward):
			"""
			main algorithm for linking track 

			Attributes: 
				nearest -- first edge on route
				i -- global iterator in search
				debug -- flag for output more information
				backward -- flag for reverse search

			return:
				forward:
					i -- iterator of exit
				backward:
					[ how many step did algorithm,
					  emergency exit]

			"""

			request = True
			backFlag = True
			while ((i < end) or (backward and backFlag)) and i < MAX:
				if backward:
					if i-first >= maxDeep - 1:
						backFlag = False
				if request:
					candidateEdge = db.getNextEdge(nearest.trgid, backward)
					if backward and backFlag:
						if len(db.getNextEdge(nearest.srcid, False)) > 1:
							backFlag = False
					if nearest.id in candidateEdge:
						candidateEdge.remove(nearest.id)
					candidateEdge.append(nearest)
					request = False
				candidats = []

				for edge in candidateEdge:
					p = P(pointList[i], edge, pointOnEdge[-1][0], nearest, pointList[i-1], 1, pointOnEdge, nearestList)
					candidats.append([p.multiProb, p])
					if debug:
						print(p.observeProb)
						print(p.angleProb)
						print(p.transmissionProb)
						print(p.multiProb)
						print(p.candidateEdge.id)
						print("-------")

				candidate = mmax(candidats)[1]
				if candidate.observeProb < propability:
					if backward == False:
						return i
					else:
						return [i - first, False]

				if debug: print(mmax(candidats)[0])
				if candidate.u < 0 or candidate.u > 1:
					"""if candidate not be over edge"""
					if nearestList[-1].id != candidate.candidateEdge.id:
						nearestList.append(candidate.candidateEdge)
						nearest = candidate.candidateEdge
						request = True

					else:
						pointOnEdge.append([candidate.candidate, candidate.candidateEdge.id, i])
						if debug: print(candidate.multiProb)
						i += 1
				else:
					pointOnEdge.append([candidate.candidate, candidate.candidateEdge.id, i])
					if nearestList[-1].id != candidate.candidateEdge.id:
						nearestList.append(candidate.candidateEdge)
						nearest = candidate.candidateEdge
						request = True
					if debug: print(candidate.multiProb)
					i += 1
				if debug:
					print(i)
					print("-------------------")

			return [i-first, True]
		return mainStep(nearest, i, debug, backward)
		

	"""Forward search"""
	i = 0
	t = i
	masI = []
	while i <= len(pointList):
		try:
			t = i
			i = search(i,len(pointList))
			masI.append(i)
			if i == t:
				i+=1
			if i == None or type(i) == list:
				print("Full search")
				masI.pop()
				break
		except IndexError:
			print('IndexError on '+str(i)+' step')
			break
		except IOError:
			print('bad bad edge')
			i += 1
		except TypeError:
			print(i)
			print('NoneType error(Normal Exit)')
			break	
		# except:
			# print('Unknown Error')
			# break


	"""Analysis forward search"""
	# print("["+",".join([str(x) for x in nearestList])+"]")
	#print(len(pointList))
	"""block for added missing point on newRoad"""
	print(len(pointOnEdge))
	print(len(pointList))
	s = 0 
	try:
		for i in range(1,len(pointList)):
			s = i
			if pointOnEdge[i][2]-pointOnEdge[i-1][2] > 1:
				for j in range(pointOnEdge[i][2]-pointOnEdge[i-1][2] - 1):
					pointOnEdge.insert(i+j,[Point(0,0),0,i+j])
	except:
		print("----=errr=----")
		print(s)
		print(len(pointOnEdge))

		raise ArithmeticError
	""""Add flag on point for checking forward or backward search"""
	for i in pointOnEdge:
		i.append(True)

	"""block for definition of range miss linking"""
	k = 1
	rangeMissLinking = []
	last = masI[0]
	flag = True
	while k <= len(masI):
		try:
			if masI[k] - masI[k-1] > 15:
				if flag:
					last = masI[k-1]
				rangeMissLinking.append([last,masI[k-1]])
				flag = True
			else:
				if flag:
					last = masI[k-1]
					flag = False
			k+=1
		except IndexError:
			if flag == False:
				rangeMissLinking.append([last, masI[-1]])
			break
	rangeMissLinking[-1][1] = len(pointList)

	"""block for clasterisation on new road and miss linking"""
	i = 0
	missLinking = []
	newRoad = []
	while i < len(rangeMissLinking):
		if rangeMissLinking[i][1] - rangeMissLinking[i][0] > 10:
			newRoad.append(rangeMissLinking[i])
		else:
			missLinking.append(rangeMissLinking[i])
		i+=1

	"""block for checking free space between two new road"""
	iter = 1
	checkingNewRoad = [newRoad[0]]
	while iter < len(newRoad):
		if newRoad[iter][0] - newRoad[iter-1][1] < 50:
			checkingNewRoad.pop()
			checkingNewRoad.append([newRoad[iter-1][0],newRoad[iter][1]])
		else:
			checkingNewRoad.append(newRoad[iter])
		iter+=1

	"""prepare dor add new road to DB"""
	poolInsertNewRoad = []
	
	"""block for delete bad edge"""
	setT = []
	for road in checkingNewRoad:
		for i in range(road[0],road[1]):
			poolInsertNewRoad.append((str(transpid)+"\t"+str(pointList[i].lat)+"\t"+str(pointList[i].lon)+"\t"+str(int(pointList[i].time))+"\n").encode('utf-8'))
			pointOnEdge[i] = [Point(0,0),0,i]
			setT.append(i)

	"""add new road to DB"""
	db.setNewRoad(poolInsertNewRoad)

	# print('checkingNewRoad: '+str(checkingNewRoad))
	# print('count in % new Road: '+str(sum([x[1]-x[0] for x in checkingNewRoad])/len(pointList)*100))
	# print('missLinking: '+str([x for x in missLinking]))

	""" Backward search """

	pointList.reverse()

	"""reverse missLinking"""
	saveNearestList = nearestList.copy()
	savePointOnEdge = pointOnEdge.copy()
	saveMissLinking = [[x[0], x[1]] for x in missLinking]
	nearestList = []
	pointOnEdge = []
	missLinking.reverse()
	for miss in missLinking:
		x = miss[1]
		miss[1] = len(pointList) - 1 - miss[0]
		miss[0] = len(pointList) - 1 - x

	"""main step of backward search"""
	retI = []
	missBackIterator = 0
	for miss in missLinking:
	
		try:
			i = search(miss[0],miss[1]+deep, True)
			retI.append(i)

		except IndexError:
			print(miss)
			missBackIterator += 1
			print('Normal Exit')
		except TypeError:
			print(miss)
			missBackIterator += 1
			print('TypeError')
		# except:
		# 	print('NotNormal Exit')

	if missBackIterator != 0:
		missLinking = missLinking[:-missBackIterator]
		saveMissLinking = saveMissLinking[missBackIterator:]

	"""Insert backsearch result"""
	for road in retI:
		if road[0] == maxDeep:
			road[1] = False

	iter = 0
	retI.append([0,False])
	retI.reverse()
	for missIter in range(len(missLinking)-1,0,-1):
		if retI[missIter][1] == False:
			iter += retI[missIter][0]
			for i in range(saveMissLinking[missIter][1]-deep,saveMissLinking[missIter][1]):
				savePointOnEdge[i] = [Point(0,0),0,i]
		else:
			#savePointOnEdge[saveMissLinking[missIter][1]-retI[missIter][0]-1] = [Point(0,0),0,saveMissLinking[missIter][1]-retI[missIter][0]-1]
			for k, i in enumerate(range(saveMissLinking[missIter][1]-retI[missIter][0],saveMissLinking[missIter][1])):
				# print(pointOnEdge[iter + k])
				pointOnEdge[iter + k][2] = i
				savePointOnEdge[i] = pointOnEdge[iter+k]
				savePointOnEdge[i].append(False)
				
			# print(missIter)
			# print("----------------")

			iter += retI[missIter][0]


	"""calculate speeds on edges"""

	"""
	1) Если на 1 ребре -> просто скорость на ребре = distance/(time2 - time1)
	2) Если на разных ребрах 

	"""
	# i = 1
	# fromLen = 0
	# toLen = 0
	# speeds = []
	# temp = savePointOnEdge[0]
	# for edge in saveNearestList:
	# 	if edge.id == savePointOnEdge[i][1] and temp[1] != edge.id:
	# 		temp = savePointOnEdge[i]
	# 		while True:
	# 			i+=1
	# 			if temp[1] != savePointOnEdge[i][1]:
	# 				if savePointOnEdge[i][1] == 0 or savePointOnEdge[i][3] == False:
	# 					if temp != None:
	# 						speeds.append([edge.id, math.round(distance(temp[0], savePointOnEdge[i-1][0])*111.1111*3.6/(savePointOnEdge[i][0].time - temp[0].time))])
	# 						temp = None
	# 						fromLen = 0
	# 				else:
	# 					if fromLen != 0:
	# 						toLen = distance(edge.src, savePointOnEdge[i][0])
	# 						speed = (toLen + fromLen)/(savePointOnEdge[i][0].time - temp[0].time)
	# 					else:
	# 						speed = 0

	# 					speeds.append([edge.id, math.round((distance(temp[0], savePointOnEdge[i-1][0])*111.1111*3.6/(savePointOnEdge[i][0].time - temp[0].time)+speed)/2)])
	# 					fromLen = distance(savePointOnEdge[i-1][0], edge.trg)
	# 					break
	# 	else:
	# 		dist = distance(temp[0], savePointOnEdge[i-1][0]) + fromLen
	# 		speeds.append([edge.id, ])


	# i = 1
	# speeds = []
	# temp = savePointOnEdge[0]
	# for edge in saveNearestList:
	# 	if edge.id == savePointOnEdge[i][1] and temp[1] != edge.id:
	# 		temp = savePointOnEdge[i]
	# 		while True:
	# 			i+=1
	# 			if temp[1] != savePointOnEdge[i][1]:
	# 				if savePointOnEdge[i][1] == 0 or savePointOnEdge[i][3] == False:
	# 					if temp != None:
	# 						speeds.append([edge.id, math.round(distance(temp[0], savePointOnEdge[i-1][0])*111.1111*3.6/(savePointOnEdge[i][0].time - temp[0].time))])
	# 						temp = None
	# 				else:
	# 					speeds.append([edge.id, math.round(distance(temp[0], savePointOnEdge[i-1][0])*111.1111*3.6/(savePointOnEdge[i][0].time - temp[0].time))])
	# 					break
	# 	else:																																																																																																																																				
	# 		dist = distance(temp[0], savePointOnE																			dge[i-1][0]) + fromLen
	# 		speeds.append([edge.id, ])
	pointList.reverse()
	i = 1

	while i < len(pointList) - 1:
		speed1 = distance(pointList[i-1],pointList[i])/(pointList[i].time - pointList[i-1].time)*111.1111*3.6*1000
		speed2 = distance(pointList[i],pointList[i+1])/(pointList[i+1].time - pointList[i].time)*111.1111*3.6*1000
		savePointOnEdge[i][0].speed = (speed1 + speed2)/2
		i+=1

	i = 2
	temp = savePointOnEdge[1][1]
	speeds = []
	speed = savePointOnEdge[1][0].speed
	l = 1
	while i < len(pointList) - 1:
		if savePointOnEdge[i][1] == temp:
			speed += savePointOnEdge[i][																																					0].speed
			l += 1
		else:
			speeds.append([savePointOnEdge[i-1][1],speed/l, transpid, savePointOnEdge[i-1][0].time])
			temp = savePointOnEdge[i][1]
			speed = savePointOnEdge[i][0].speed
			l = 1
		i += 1

	"""add speed to DB"""
	insertSpeedPool = []
	for speed in speeds:
		if speed[3] != None:
			insertSpeedPool.append((str(int(speed[0]))+"\t"+str(int(speed[1]))+"\t"+str(int(speed[2]))+"\t"+str(int(speed[3]))+"\n").encode('utf-8'))

	db.setNewSpeed(insertSpeedPool)

	"""screening bad edge"""
	pointOnEdge = []
	for point in savePointOnEdge:
		if point[1] != 0:
			pointOnEdge.append(point)

	"""output Point linking to edge"""
	#print("["+",".join([str(x[0].speed) for x in pointOnEdge[1800:3010]])+"]")

	"""output route on edges"""
	# nearestList = [[db.getRoute(pointOnEdge[0][1])]]
	# x = 1
	# for x in range(len(pointOnEdge)):
	# 	if pointOnEdge[x-1][1] != pointOnEdge[x][1]:
	# 		if pointOnEdge[x-1][2] - pointOnEdge[x][2] > 3:
	# 			nearestList.append([])
	# 		nearestList[-1].append(db.getRoute(pointOnEdge[x][1]))
	# a = "["
	# for y in nearestList:																																																																																																																
	# 	a+= "["+",".join([str(x) for x in y])+"],"	
	# a+="]"
	# print(a)

for task in db.getTransports():
	main(task[0], task[1])

main(191,0)
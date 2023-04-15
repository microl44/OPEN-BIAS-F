import csv
import pandas as pd
import numpy as np
import math
import itertools
import matplotlib.pyplot as plt
import scipy.stats as st

def avg(inputList):
	return sum(inputList) / float(len(inputList))

def conf(inputList):
	return st.t.interval(confidence=0.95, df=len(inputList)-1, loc=np.mean(inputList), scale=st.sem(inputList))


df = pd.read_csv('result.csv', delimiter=';')

time_dict = {'dijkstra': {'complete': {'static': [], '1': [], '2': [], '3': []}, 
							'mapless': {'static': [], '1': [], '2': [], '3': []}}, 
			'a-star': {'complete': {'static': [], '1': [], '2': [], '3': []}, 
						'mapless': {'static': [], '1': [], '2': [], '3': []}}}

distance_dict = {'dijkstra': {'complete': {'static': [], '1': [], '2': [], '3': []}, 
								'mapless': {'static': [], '1': [], '2': [], '3': []}}, 
				'a-star': {'complete': {'static': [], '1': [], '2': [], '3': []}, 
							'mapless': {'static': [], '1': [], '2': [], '3': []}}}

std_dict = {'dijkstra': {'complete': {'static': [], '1': [], '2': [], '3': []}, 
							'mapless': {'static': [], '1': [], '2': [], '3': []}}, 
			'a-star': {'complete': {'static': [], '1': [], '2': [], '3': []}, 
						'mapless': {'static': [], '1': [], '2': [], '3': []}}}

confidence_interval_dict = {'dijkstra': {'complete': {'static': [], '1': [], '2': [], '3': []}, 
								'mapless': {'static': [], '1': [], '2': [], '3': []}}, 
				'a-star': {'complete': {'static': [], '1': [], '2': [], '3': []}, 
							'mapless': {'static': [], '1': [], '2': [], '3': []}}}

#add stuff to appropriate dictionary list
for i, row in df.iterrows():
	if(row["Error"] == "Error"):
		pass
	else:
		algorithm = row['Algorithm']
		map_value = row['Map']
		environment = row['Environment']
		time = row['Time (s)']
		distance = row['Distance (m)']
		time_dict[algorithm][map_value][environment].append(time)
		distance_dict[algorithm][map_value][environment].append(distance)
	
for algorithm in time_dict:
	for map_value in time_dict[algorithm]:
		for environment in time_dict[algorithm][map_value]:
			time_list = time_dict[algorithm][map_value][environment]
			distance_list = distance_dict[algorithm][map_value][environment]
			
			#fig, axes = plt.subplots(6, 10, figsize=(30,30))
			#print(f"Algorithm: {algorithm}, Map: {map_value}, Environment: {environment}")
			#for i in range(len(time_list)):
				#print(f"Time: {time_list[i]}, Distance: {distance_list[i]}")

#present graphs regarding distance
def GraphDistance():
	plotX = [1, 2, 3, 4]
	ylim = [14, 20]
	MD = [avg(distance_dict["dijkstra"]["complete"]["static"]), avg(distance_dict["dijkstra"]["complete"]["1"]), avg(distance_dict["dijkstra"]["complete"]["2"]), avg(distance_dict["dijkstra"]["complete"]["3"])]
	MA = [avg(distance_dict["a-star"]["complete"]["static"]), avg(distance_dict["a-star"]["complete"]["1"]), avg(distance_dict["a-star"]["complete"]["2"]), avg(distance_dict["a-star"]["complete"]["3"])]
	UD = [avg(distance_dict["dijkstra"]["mapless"]["static"]), avg(distance_dict["dijkstra"]["mapless"]["1"]), avg(distance_dict["dijkstra"]["mapless"]["2"]), avg(distance_dict["dijkstra"]["mapless"]["3"])]
	UA = [avg(distance_dict["a-star"]["mapless"]["static"]), avg(distance_dict["a-star"]["mapless"]["1"]), avg(distance_dict["a-star"]["mapless"]["2"]), avg(distance_dict["a-star"]["mapless"]["3"])]
	plotLabels = ["Static", "Low", "Medium", "High"]

	fig, axs = plt.subplots(2, 2)
	fig.suptitle('Results: Distance (m)')
	axs[0,0].bar(plotX, MD, width=0.3, color=['red', 'green', 'blue', 'black'])
	axs[0,0].set_title("Mapped Dijkstra")
	axs[0,0].set_ylim(ylim)
	axs[0,1].bar(plotX, UD, width=0.3, color=['red', 'green', 'blue', 'black'])
	axs[0,1].set_title("Unmapped Dijkstra")
	axs[0,1].set_ylim(ylim)
	axs[1,0].bar(plotX, MA, width=0.3, color=['red', 'green', 'blue', 'black'])
	axs[1,0].set_title("Mapped A-STAR")
	axs[1,0].set_ylim(ylim)
	axs[1,1].bar(plotX, UA, width=0.3, color=['red', 'green', 'blue', 'black'])
	axs[1,1].set_title("Unmapped A-STAR")
	axs[1,1].set_ylim(ylim)

	plt.setp(axs, xticks=[1, 2, 3, 4], xticklabels=plotLabels)
	plt.show()

#present graphs regarding time
def GraphTime():
	plotX = [1, 2, 3, 4]
	ylim = [80, 120]
	MD = [avg(time_dict["dijkstra"]["complete"]["static"]), avg(time_dict["dijkstra"]["complete"]["1"]), avg(time_dict["dijkstra"]["complete"]["2"]), avg(time_dict["dijkstra"]["complete"]["3"])]
	MA = [avg(time_dict["a-star"]["complete"]["static"]), avg(time_dict["a-star"]["complete"]["1"]), avg(time_dict["a-star"]["complete"]["2"]), avg(time_dict["a-star"]["complete"]["3"])]
	UD = [avg(time_dict["dijkstra"]["mapless"]["static"]), avg(time_dict["dijkstra"]["mapless"]["1"]), avg(time_dict["dijkstra"]["mapless"]["2"]), avg(time_dict["dijkstra"]["mapless"]["3"])]
	UA = [avg(time_dict["a-star"]["mapless"]["static"]), avg(time_dict["a-star"]["mapless"]["1"]), avg(time_dict["a-star"]["mapless"]["2"]), avg(time_dict["a-star"]["mapless"]["3"])]
	plotLabels = ["Static", "Low", "Medium", "High"]

	fig, axs = plt.subplots(2, 2)
	fig.suptitle('Results: Time (s)')
	axs[0,0].bar(plotX, MD, width=0.3, color=['red', 'green', 'blue', 'black'])
	axs[0,0].set_title("Mapped Dijkstra")
	axs[0,0].set_ylim(ylim)
	axs[0,1].bar(plotX, UD, width=0.3, color=['red', 'green', 'blue', 'black'])
	axs[0,1].set_title("Unmapped Dijkstra")
	axs[0,1].set_ylim(ylim)
	axs[1,0].bar(plotX, MA, width=0.3, color=['red', 'green', 'blue', 'black'])
	axs[1,0].set_title("Mapped A-STAR")
	axs[1,0].set_ylim(ylim)
	axs[1,1].bar(plotX, UA, width=0.3, color=['red', 'green', 'blue', 'black'])
	axs[1,1].set_title("Unmapped A-STAR")
	axs[1,1].set_ylim(ylim)

	plt.setp(axs, xticks=[1, 2, 3, 4], xticklabels=plotLabels)
	plt.show()

#print time std and 95% conf interval
def printTimeStats():
	for algorithm in time_dict:
		for map_value in time_dict[algorithm]:
			for environment in time_dict[algorithm][map_value]:
				std_dict[algorithm][map_value][environment].append(np.std(time_dict[algorithm][map_value][environment]))
				print(algorithm + " " + map_value + " " + environment + " Standard Deviation: " + str(np.std(time_dict[algorithm][map_value][environment])))
				print(algorithm + " " + map_value + " " + environment + " 95% confidence interval: " + str(conf(time_dict[algorithm][map_value][environment])))
				print("")

#print distance std and 95% conf interval
def printDistanceStats():
	for algorithm in distance_dict:
		for map_value in distance_dict[algorithm]:
			for environment in distance_dict[algorithm][map_value]:
				confidence_interval_dict[algorithm][map_value][environment].append(np.std(time_dict[algorithm][map_value][environment]))
				print(algorithm + " " + map_value + " " + environment + " Standard Deviation: " + str(np.std(distance_dict[algorithm][map_value][environment])))
				print(algorithm + " " + map_value + " " + environment + " 95% confidence interval: " + str(conf(distance_dict[algorithm][map_value][environment])))
				print("")

#COMMENT OUT THE FUNCTION YOU'D LIKE TO USE!!!!!!!
#GraphTime()
#GraphDistance()
#printTimeStats()
#printDistanceStats()
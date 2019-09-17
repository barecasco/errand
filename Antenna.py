
import string
import numpy as np
import random
import math
import copy
import argparse
import json
import pandas as pd
import re
import time
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

import warnings
warnings.filterwarnings("ignore")
alphabet = list(string.ascii_lowercase)

import requests

def request_sync_time_matrix(locfile):
    bingkey = "AkLAgpAMiqj6EIoeIWu7H_ld13VmTfnZPh4i7nrObAGF8CfWmRRRjGcgXfaAW5pp"
    url = "https://dev.virtualearth.net/REST/v1/Routes/DistanceMatrix?key="+bingkey
    
    headers = {
        'Content-Type' : 'application/json'
    }

    separator = ","
    
    loc_df = pd.read_csv(locfile, sep=separator)
    origins = [{"latitude" : loc_df['lat'][i], "longitude" : loc_df['lon'][i]} for i in range(len(loc_df))]
    destinations = [{"latitude" : loc_df['lat'][i], "longitude" : loc_df['lon'][i]} for i in range(len(loc_df))]
    body = {
        "travelMode" : "driving",
        "origins" : origins,
        "destinations" : destinations
    }
    
    input_data = json.dumps(body)
    res = requests.post(url, input_data, headers=headers)
    resjson = json.loads(res.text)
    time_frame = resjson['resourceSets'][0]['resources'][0]['results']    
    time_df = pd.DataFrame(time_frame).astype('int')
    ctable = time_df.pivot(index='originIndex', columns='destinationIndex', values='travelDuration')
    ctable[0] = 0
    return ctable.values
    
    
def create_distance_callback(data, manager):
    distances_ = data['time_matrix']
    index_manager_ = manager
    
    def distance_callback(from_index, to_index):
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = index_manager_.IndexToNode(from_index)
        to_node = index_manager_.IndexToNode(to_index)
        return distances_[from_node][to_node]

    return distance_callback

def get_farthest_index(matrix):
    depo_routes = matrix[0]
    max_distance = np.amax (depo_routes)
    res = np.where(depo_routes == max_distance)
    return (res[0][0])

def print_solution(data, manager, routing, assignment):
    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0
    callsigns = data['callsigns']
    route_callsigns = []
    timestamps = []
    
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = '\n'
        
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            tid = manager.IndexToNode(index)
            timestamp = assignment.Max(time_var)
            
            route_callsigns.append(data['callsigns'][tid])
            timestamps.append(timestamp)
            
            plan_output += '{0} Time({1}) -> '.format(callsigns[tid], timestamp)
            index = assignment.Value(routing.NextVar(index))
        plan_output += "END"
        time_var = time_dimension.CumulVar(index)

        print(plan_output)
        total_time += assignment.Min(time_var)
    
    print('Total time of all routes: {}min'.format(total_time))
    return(route_callsigns, timestamps)

def transform_kml_to_csv(kmlfile, csvfile, solution_id):
    targetpoints = []
    f= open(kmlfile,"r")
    kml = str(f.read())
    kml = kml.replace('\n', '')
    coordinates = re.search(r'<coordinates>(.*?)</coordinates>', kml).group(1).strip()
    coordinates = re.sub(r"\s+", ';', coordinates)
    coordinates = coordinates.split(";")
    for coordstring in coordinates:
        loc = []
        for el in coordstring.split(","):
            loc.append(float(el))
        targetpoints.append(loc)    
    df = pd.DataFrame(targetpoints, columns=['lon', 'lat', 'height'])
    df['drv'] = solution_id
    locs = ["depo"]
    for i in range(1, len(df)):
        locs.append("tgt"+str(i))
    df["loc"] = locs
    df = df.iloc[:, [4, 1, 0, 3]]
    df.to_csv(csvfile)
    return(df)

def transform_list_to_csv(coordinates, csvfile, solution_id):
    targetpoints = []
    for pair in coordinates:
        loc = []
        for el in pair:
            loc.append(float(el))
        loc.append(0)
        targetpoints.append(loc)    
        
    df = pd.DataFrame(targetpoints, columns=['lat', 'lon', 'height'])
    df['drv'] = solution_id
    locs = ["depo"]
    for i in range(1, len(df)):
        locs.append("tgt"+str(i))
    df["loc"] = locs
    df = df.iloc[:, [4, 0, 1, 3]]
    df.to_csv(csvfile)
    return(df)

def search_solution ( data ):
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)
    distance_callback = create_distance_callback(data, manager)
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    routing.AddDimension(
        transit_callback_index,
        240,  # allowed waiting time
        240,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        'Time')

    time_dimension = routing.GetDimensionOrDie('Time')
    # -------------------------------------------- Add time window
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for each vehicle start node.
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
                                                data['time_windows'][0][1])
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))
    # -------------------------------------------- end time window
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    assignment = routing.SolveWithParameters(search_parameters)
    
    if assignment:
        print("\nSOLUTION FOUND ON ROUTE:\n---------------------")
        solution_callsigns, timestamps = print_solution(data, manager, routing, assignment)
        data['solution_callsigns'] = solution_callsigns
        data['solution_timestamps'] = timestamps
    else:
        farthest_index = get_farthest_index(data['time_matrix'])
        if (farthest_index == 0) :
            print("NO SOLUTION FOUND.")
            return
        
        data['time_matrix'] = np.delete(data['time_matrix'], farthest_index, axis=0)
        data['time_matrix'] = np.delete(data['time_matrix'], farthest_index, axis=1)        
        data['time_windows'].pop(farthest_index)
        data['callsigns'].pop(farthest_index)
        search_solution (data)

def request_best_route(locfile,  solution_name):
    locdf = pd.read_csv(locfile,sep=",")
    
    # --- get time matrix
    ctable = request_sync_time_matrix(locfile)
    package_num = len(ctable) - 1
    print("Time Table:\n\n", ctable)
    # --- create data
    data = {}
    data['time_matrix'] = ctable
    # assume all targets wait for 120 mins
    data['time_windows'] = [[0, 120] for i in range(package_num)]
    data['callsigns'] = ['depo']
    for i in range(1, package_num+1):
        data['callsigns'].append("tgt"+str(i))
    data['num_vehicles'] = 1
    data['depot'] = 0

    # ---------------------- Display time window
    start_time = time.time()
    search_solution(data)
    print("--- %s seconds ---" % (time.time() - start_time))
    
    # create result table
    locs = list(locdf['loc'])
    solution_indices = [locs.index(cs) for cs in data['solution_callsigns']]

    solution_lats = locdf['lat'][solution_indices].values
    solution_lons = locdf['lon'][solution_indices].values

    soldf = pd.DataFrame({
        'stamp':data['solution_timestamps'],
        'loc':data['solution_callsigns'],
        'lat':solution_lats,
        'lon':solution_lons,
        'drv':solution_name
    })    
    
    return(soldf)
        
def request_waypoints(soldf, solution_name):
    # create waypoints
    waypoints = [[soldf['lat'][i], soldf['lon'][i]] for i in range(len(soldf))]
    bingkey = "AkLAgpAMiqj6EIoeIWu7H_ld13VmTfnZPh4i7nrObAGF8CfWmRRRjGcgXfaAW5pp"
    url = "http://dev.virtualearth.net/REST/V1/Routes/Driving"
    pointschain = []

    for i in range(len(waypoints)-1):
        waypars = {}
        waypars["wp.0"] = str(waypoints[i][0]) +","+str(waypoints[i][1])
        waypars["wp.1"] = str(waypoints[i+1][0]) +","+str(waypoints[i+1][1])

        opars = {
            'optmz':'distance',
            'ra': 'RoutePath',
            'rpo': 'Points',
            'key':bingkey
        }

        opars.update(waypars)
        node = requests.get(url, params = opars).json()
        points = node['resourceSets'][0]['resources'][0]['routePath']['line']['coordinates']

        for point in points:
            point.append("trip "+str(i+1))
            point.append(solution_name)

        pointschain.append(points)

    waychain = np.concatenate(pointschain, 0)
    wpdf = pd.DataFrame(waychain, columns = ['lat', 'lon', 'seg', 'drv'])
    return(wpdf)   

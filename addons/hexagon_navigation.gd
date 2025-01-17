extends Node

#Method documentation: https://docs.google.com/document/d/1HwLlRmC2tDGbadkOEero5asVq6XmzpupeJX_pUEwqGg/edit?usp=sharing

var current_map : TileMapLayer
var astar = AStar2D.new()

func set_current_map(map : TileMapLayer):
	current_map = map
	if current_map != null:
		astar.clear()
		add_all_point()
	print("there are " + str(astar.get_point_count()) + " points in this map")
	
func add_all_point(): #add and connect all cells
	var all_used_cells = current_map.get_used_cells()
	for cell in all_used_cells:
		astar.add_point(astar.get_available_point_id(), cell)
	for point_id in astar.get_point_ids():
		var pos = astar.get_point_position(point_id)
		var all_possible_neighbors = current_map.get_surrounding_cells(pos)
		var valid_neighbor = []
		for neighbor in all_possible_neighbors:
			if current_map.get_cell_source_id(neighbor) != -1: #if the cell is not empty
				valid_neighbor.append(neighbor)
		for neighbor in valid_neighbor:
			var neighbor_id = astar.get_closest_point(neighbor)
			astar.connect_points(point_id, neighbor_id)

#general process of converting global position to a cell position
	#1. convert global position to map node's local
	#2. convert map node's local to map coordinates
	#3. use the map coordinate to locate the closest cell in astar
	#do the other way around to convert cell position to global position
	
func cell_to_global(cell_pos : Vector2i) -> Vector2: #returns another global position
	return current_map.to_global(current_map.map_to_local(cell_pos))

func global_to_cell(global_pos : Vector2) -> Vector2i: #returns local cell position
	var closest_point_id = astar.get_closest_point(current_map.local_to_map(current_map.to_local(global_pos))) #TODO: Fix bug where clicking on an empty map will register as the closest tile
	return astar.get_point_position(closest_point_id)

func get_cell_custom_data(cell_pos: Vector2i, data_name: String):
	var data = current_map.get_cell_tile_data(cell_pos)
	if data:
		return data.get_custom_data(data_name)
	else:
		return
	
func get_navi_path(start_pos : Vector2i, end_pos : Vector2i) -> PackedVector2Array: #returns an array of cell positions from start to goal
	var start_id = tile_to_id(start_pos)
	var goal_id = tile_to_id(end_pos)
	var path_taken = astar.get_point_path(start_id, goal_id)
	return path_taken
	
func show_path(path : Array[Vector2i]): #highlights cells of a given path; debugging function
	for index in range(0, path.size()):
		if index == 0 or index == path.size()-1:
			current_map.set_cell_to_variant(1, path[index]) #variant depends on each map layer
		else:
			current_map.set_cell_to_variant(2, path[index])

func clear_path(): #clear all markings on the map; debugging function
	var all_cells = current_map.get_used_cells()
	for cell in all_cells:
		current_map.set_cell_to_variant(0, cell)

func tile_to_id(pos: Vector2i) -> int: #using a tile position to get the id for AStar usage
	#assuming that all available tiles are already mapped in astar
	if current_map.get_cell_source_id(pos) != -1:
		return astar.get_closest_point(pos)
	else: return -1

func id_to_tile(id: int) -> Vector2i:
	if astar.has_point(id):
		return astar.get_point_position(id)
	return Vector2i(-1, -1)

func get_distance(pos1: Vector2i, pos2: Vector2i) -> int:
	var all_points = get_navi_path(pos1, pos2)
	return all_points.size() - 1 #excluding the first point

func get_all_neighbors_in_range(start_pos: Vector2i, range: int) -> Array[Vector2i]:
	#returns an array of cell ID
	#employs a depth first search
	var all_neighbors_id : Array[int] = []
	var starting_cell_id = tile_to_id(start_pos)
	_dfs(range, starting_cell_id, starting_cell_id, all_neighbors_id)
	var all_neighbors_pos = all_neighbors_id.map(id_to_tile)
	var answer: Array[Vector2i]
	answer.assign(all_neighbors_pos)
	#print(answer, start_pos)
	return answer
	
func _dfs(k : int, node_id : int, parent_id : int, solution_arr : Array): #helper recursive function
	#godot seems to pass array by reference by default
	if k < 0 or node_id == -1:
		return
	if !solution_arr.has(node_id):
		solution_arr.append(node_id)
	for neighbor_pos in current_map.get_surrounding_cells(astar.get_point_position(node_id)):
		var neighbor_id = tile_to_id(neighbor_pos)
		if neighbor_id == null:
			return
		if neighbor_id != parent_id:
			_dfs(k-1, neighbor_id, node_id, solution_arr)
	
func get_random_tile_pos() -> Vector2: #for testing and placeholder purposes
	return astar.get_point_position(randi_range(0, astar.get_point_count()))
	
func get_random_tile_from(start: Vector2i, range : int):
	pass

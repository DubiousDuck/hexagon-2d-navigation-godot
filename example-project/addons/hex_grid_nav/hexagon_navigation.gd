extends Node

#Method documentation: https://docs.google.com/document/d/1HwLlRmC2tDGbadkOEero5asVq6XmzpupeJX_pUEwqGg/edit?usp=sharing

var current_map : TileMapLayer
var astar = AStar2D.new()

## Initial function that binds a [TileMapLayer] to the navigation class
func set_current_map(map : TileMapLayer):
	current_map = map
	if current_map != null:
		astar.clear()
		add_all_point()
	print("there are " + str(astar.get_point_count()) + " points in this map")

## Adds and connects all cells in the [TileMapLayer]
func add_all_point(): 
	var all_used_cells = current_map.get_used_cells()
	for cell in all_used_cells:
		var next_id := astar.get_available_point_id()
		astar.add_point(next_id, cell)
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

## Returns the global position of a cell position
func cell_to_global(cell_pos : Vector2i) -> Vector2:
	return current_map.to_global(current_map.map_to_local(cell_pos))

## Returns the cell position of a global position; returns [code]Vector2i(-999, 999)[/code] if no valid cell is found.
func global_to_cell(global_pos : Vector2) -> Vector2i: #returns local cell position
	var local_map_pos := current_map.local_to_map(current_map.to_local(global_pos))
	var closest_point_id = astar.get_closest_point(local_map_pos)
	var closest_cell: Vector2i = astar.get_point_position(closest_point_id)
	if local_map_pos != closest_cell: #prevents returning nonexistent cell
		return Vector2i(-999, -999)
	return closest_cell 

## Returns the cell position that is closest to a given global position; similar to [method HexNavi.global_to_cell()] but always returns a valid cell position.
func get_closest_cell_by_global_pos(global_pos : Vector2) -> Vector2i:
	var local_map_pos := current_map.local_to_map(current_map.to_local(global_pos))
	var closest_point_id = astar.get_closest_point(local_map_pos)
	var closest_cell: Vector2i = astar.get_point_position(closest_point_id)
	return closest_cell

func get_cell_custom_data(cell_pos: Vector2i, data_name: String):
	var data = current_map.get_cell_tile_data(cell_pos)
	if data:
		return data.get_custom_data(data_name)
	else:
		return

## Returns an array of cell positions from start to goal
func get_navi_path(start_pos : Vector2i, end_pos : Vector2i) -> PackedVector2Array:
	var start_id = tile_to_id(start_pos)
	var goal_id = tile_to_id(end_pos)
	var path_taken = astar.get_point_path(start_id, goal_id)
	return path_taken


## Use a tile position to get the id for AStar usage
func tile_to_id(pos: Vector2i) -> int: 
	#assuming that all available tiles are already mapped in astar
	if current_map.get_cell_source_id(pos) != -1:
		return astar.get_closest_point(pos)
	else: return -1

## Use an AStar point ID to get the point's tile position
func id_to_tile(id: int) -> Vector2i:
	if astar.has_point(id):
		return astar.get_point_position(id)
	return Vector2i(-1, -1)

## Returns the number of tile between [param pos1] and [param pos2]
func get_distance(pos1: Vector2i, pos2: Vector2i) -> int:
	var all_points = get_navi_path(pos1, pos2)
	return all_points.size() - 1 #excluding the first point

## Returns all neighbor cells of a given cell at [param start_pos] within [param range].[br] Use [param traversable_only] to decide whether to include intraversable cells in search.
func get_all_neighbors_in_range(start_pos: Vector2i, range: int, traversable_only: bool = true) -> Array[Vector2i]:
	#returns an array of cell ID
	#employs a depth first search
	var all_neighbors_id : Array[int] = []
	var starting_cell_id = tile_to_id(start_pos)
	_dfs(range, starting_cell_id, starting_cell_id, all_neighbors_id, traversable_only)
	var all_neighbors_pos = all_neighbors_id.map(id_to_tile)
	var answer: Array[Vector2i]
	answer.assign(all_neighbors_pos)
	return answer
	
func _dfs(k : int, node_id : int, parent_id : int, solution_arr : Array, traversable_only : bool): #helper recursive function
	#Godot passes array by reference by default
	if k < 0 or node_id == -1:
		return
	if traversable_only:
		if !get_cell_custom_data(id_to_tile(node_id), "traversable"):
			return
	if !solution_arr.has(node_id):
		solution_arr.append(node_id)
	for neighbor_pos in current_map.get_surrounding_cells(astar.get_point_position(node_id)):
		var neighbor_id = tile_to_id(neighbor_pos)
		if neighbor_id != parent_id:
			_dfs(k-1, neighbor_id, node_id, solution_arr, traversable_only)
	
func get_random_tile_pos() -> Vector2: #for testing and placeholder purposes
	return astar.get_point_position(randi_range(0, astar.get_point_count()-1))
	
